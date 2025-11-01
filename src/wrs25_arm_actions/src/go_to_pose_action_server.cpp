#include <memory>
#include <thread>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "wrs25_arm_actions/action/go_to_pose.hpp"

namespace wrs25_arm_actions
{
class GoToPoseActionServer : public rclcpp::Node
{
public:
    using GoToPose = wrs25_arm_actions::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

    explicit GoToPoseActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("go_to_pose_action_server", options)
    {
        using namespace std::placeholders;

        // Initialize MoveGroupInterface as a shared pointer to be set up on first goal execution.
        this->action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "go_to_pose",
            std::bind(&GoToPoseActionServer::handle_goal, this, _1, _2),
            std::bind(&GoToPoseActionServer::handle_cancel, this, _1),
            std::bind(&GoToPoseActionServer::handle_accepted, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "GoToPoseActionServer has been initialized and is awaiting goals.");
    }

private:
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request. Pipeline: '%s', Planner: '%s'",
                    goal->planning_pipeline_id.c_str(),
                    goal->planner_id.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        using namespace std::placeholders;
        // Start a separate thread to execute the goal to avoid blocking the action server
        std::thread{std::bind(&GoToPoseActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        // 1. Setup MoveIt Interface (if not already done)
        if (!this->move_group_interface_) {
            // Need a Node pointer to initialize MoveGroupInterface
            auto node_ptr = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
            // Replace "ur5_arm" with your actual planning group name if different
            this->move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "ur5_arm");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized for ur5_arm group.");
        }

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<GoToPose::Result>();

        RCLCPP_INFO(this->get_logger(), "Target pose: [%.3f, %.3f, %.3f]", 
                    goal->target_pose.pose.position.x, 
                    goal->target_pose.pose.position.y, 
                    goal->target_pose.pose.position.z);

        // 2. Dynamic Planner Configuration
        
        // Set Planning Pipeline ID if provided in the goal
        if (!goal->planning_pipeline_id.empty()) {
            this->move_group_interface_->setPlanningPipelineId(goal->planning_pipeline_id);
            RCLCPP_INFO(this->get_logger(), "Set Planning Pipeline ID to: %s", goal->planning_pipeline_id.c_str());
        } else {
            // Fallback to MoveIt's default pipeline if none is specified
            RCLCPP_INFO(this->get_logger(), "Using default planning pipeline.");
        }

        // Set Planner ID if provided in the goal
        if (!goal->planner_id.empty()) {
            this->move_group_interface_->setPlannerId(goal->planner_id);
            RCLCPP_INFO(this->get_logger(), "Set Planner ID to: %s", goal->planner_id.c_str());
        } else {
            // Fallback to the default planner within the selected pipeline
            RCLCPP_INFO(this->get_logger(), "Using default planner ID within the selected pipeline.");
        }

        // 3. Set Target and Plan
        this->move_group_interface_->setPoseTarget(goal->target_pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        // Attempt planning
        if (this->move_group_interface_->plan(my_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = false;
            result->message = "Planning failed with requested configuration.";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Planning failed for target pose.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Planning succeeded.");

        // 4. Execution and Feedback Loop
        auto future = std::async(std::launch::async, [this, my_plan]() {
            return this->move_group_interface_->execute(my_plan);
        });

        rclcpp::Rate loop_rate(10); 
        auto feedback = std::make_shared<GoToPose::Feedback>();

        while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            if (goal_handle->is_canceling()) {
                this->move_group_interface_->stop();
                result->success = false;
                result->message = "Action Canceled by user request.";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "GoToPose goal canceled.");
                return;
            }

            feedback->current_pose = this->move_group_interface_->getCurrentPose();
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }

        // 5. Final Result
        moveit::core::MoveItErrorCode final_code = future.get();
        if (final_code == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true;
            result->message = "Goal reached successfully.";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal execution succeeded.");
        } else {
            result->success = false;
            result->message = "Execution failed with code: " + std::to_string(final_code.val);
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Goal execution failed.");
        }
    }
};
} // namespace wrs25_arm_actions

RCLCPP_COMPONENTS_REGISTER_NODE(wrs25_arm_actions::GoToPoseActionServer)
