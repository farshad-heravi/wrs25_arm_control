#include <memory>
#include <thread>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "wrs25_arm_actions/action/go_to_joint.hpp"

namespace wrs25_arm_actions
{
class GoToJointActionServer : public rclcpp::Node
{
public:
    using GoToJoint = wrs25_arm_actions::action::GoToJoint;
    using GoalHandleGoToJoint = rclcpp_action::ServerGoalHandle<GoToJoint>;

    explicit GoToJointActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("go_to_joint_action_server", options)
    {
        using namespace std::placeholders;

        // Initialize action server
        this->action_server_ = rclcpp_action::create_server<GoToJoint>(
            this,
            "go_to_joint",
            std::bind(&GoToJointActionServer::handle_goal, this, _1, _2),
            std::bind(&GoToJointActionServer::handle_cancel, this, _1),
            std::bind(&GoToJointActionServer::handle_accepted, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "GoToJointActionServer has been initialized and is awaiting goals.");
    }

private:
    rclcpp_action::Server<GoToJoint>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToJoint::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with %zu joint positions. Pipeline: '%s', Planner: '%s'",
                    goal->joint_positions.size(),
                    goal->planning_pipeline_id.c_str(),
                    goal->planner_id.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGoToJoint> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGoToJoint> goal_handle)
    {
        using namespace std::placeholders;
        // Start a separate thread to execute the goal to avoid blocking the action server
        std::thread{std::bind(&GoToJointActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleGoToJoint> goal_handle)
    {
        // 1. Setup MoveIt Interface (if not already done)
        if (!this->move_group_interface_) {
            auto node_ptr = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
            this->move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "ur5_arm");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized for ur5_arm group.");
        }

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<GoToJoint::Result>();

        // 2. Validate joint positions count
        size_t num_joints = this->move_group_interface_->getJointNames().size();
        if (goal->joint_positions.size() != num_joints) {
            result->success = false;
            result->message = "Invalid number of joint positions. Expected " + 
                             std::to_string(num_joints) + " but got " + 
                             std::to_string(goal->joint_positions.size());
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            return;
        }

        // Log target joint positions
        std::stringstream ss;
        ss << "Target joint positions: [";
        for (size_t i = 0; i < goal->joint_positions.size(); ++i) {
            ss << goal->joint_positions[i];
            if (i < goal->joint_positions.size() - 1) ss << ", ";
        }
        ss << "]";
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

        // 3. Clear previous targets
        this->move_group_interface_->clearPoseTargets();
        RCLCPP_INFO(this->get_logger(), "Cleared previous targets");

        // 4. Dynamic Planner Configuration
        
        // Set Planning Pipeline ID if provided in the goal
        if (!goal->planning_pipeline_id.empty()) {
            this->move_group_interface_->setPlanningPipelineId(goal->planning_pipeline_id);
            RCLCPP_INFO(this->get_logger(), "Set Planning Pipeline ID to: %s", goal->planning_pipeline_id.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Using default planning pipeline.");
        }

        // Set Planner ID if provided in the goal
        if (!goal->planner_id.empty()) {
            this->move_group_interface_->setPlannerId(goal->planner_id);
            RCLCPP_INFO(this->get_logger(), "Set Planner ID to: %s", goal->planner_id.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Using default planner ID within the selected pipeline.");
        }

        // 5. Set Target Joint Values and Plan
        this->move_group_interface_->setJointValueTarget(goal->joint_positions);
        
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        // Attempt planning
        if (this->move_group_interface_->plan(my_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = false;
            result->message = "Planning failed with requested configuration.";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Planning failed for target joint configuration.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Planning succeeded.");

        // 6. Execution and Feedback Loop
        auto future = std::async(std::launch::async, [this, my_plan]() {
            return this->move_group_interface_->execute(my_plan);
        });

        rclcpp::Rate loop_rate(10); 
        auto feedback = std::make_shared<GoToJoint::Feedback>();

        while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            if (goal_handle->is_canceling()) {
                this->move_group_interface_->stop();
                result->success = false;
                result->message = "Action Canceled by user request.";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "GoToJoint goal canceled.");
                return;
            }

            // Get current joint values for feedback
            feedback->current_joint_positions = this->move_group_interface_->getCurrentJointValues();
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }

        // 7. Final Result
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

RCLCPP_COMPONENTS_REGISTER_NODE(wrs25_arm_actions::GoToJointActionServer)

