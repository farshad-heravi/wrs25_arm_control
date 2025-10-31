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

        this->action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "go_to_pose",
            std::bind(&GoToPoseActionServer::handle_goal, this, _1, _2),
            std::bind(&GoToPoseActionServer::handle_cancel, this, _1),
            std::bind(&GoToPoseActionServer::handle_accepted, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "GoToPoseActionServer has been initialized.");
    }

private:
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        (void)goal; // Mark as unused to prevent compiler warnings
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
        std::thread{std::bind(&GoToPoseActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal with std::async for feedback");
        if (!this->move_group_interface_) {
            auto node_ptr = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
            this->move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "ur5_arm");
            
            // Configuration is now handled by MoveIt config files
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized for ur5_arm group");
            RCLCPP_INFO(this->get_logger(), "Planning frame: %s", this->move_group_interface_->getPlanningFrame().c_str());
            RCLCPP_INFO(this->get_logger(), "End effector link: %s", this->move_group_interface_->getEndEffectorLink().c_str());
        }

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<GoToPose::Result>();

        RCLCPP_INFO(this->get_logger(), "Target pose: [%.3f, %.3f, %.3f]", 
                    goal->target_pose.pose.position.x, 
                    goal->target_pose.pose.position.y, 
                    goal->target_pose.pose.position.z);

        // Stage 1: Use OMPL to find a collision-free IK solution for the pose goal.
        RCLCPP_INFO(this->get_logger(), "Stage 1: Planning with OMPL to find a valid joint goal.");
        this->move_group_interface_->setPlanningPipelineId("ompl");
        this->move_group_interface_->setPoseTarget(goal->target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan ompl_plan;
        if (this->move_group_interface_->plan(ompl_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = false;
            result->message = "OMPL failed to find a collision-free path to the target pose.";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "OMPL planning failed.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "OMPL planning succeeded. Extracting joint goal.");

        // Extract the joint values from the end of the OMPL plan.
        std::vector<double> chomp_joint_goal = ompl_plan.trajectory_.joint_trajectory.points.back().positions;

        // Stage 2: Use CHOMP to plan a smooth trajectory to the joint goal found by OMPL.
        RCLCPP_INFO(this->get_logger(), "Stage 2: Planning with CHOMP to the OMPL-found joint goal.");
        this->move_group_interface_->setPlanningPipelineId("chomp"); // Assuming your default is chomp, but being explicit is good.
        this->move_group_interface_->setJointValueTarget(chomp_joint_goal);
        
        moveit::planning_interface::MoveGroupInterface::Plan my_plan; // This will be the final CHOMP plan
        if (this->move_group_interface_->plan(my_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = false;
            result->message = "CHOMP planning failed to the OMPL-found joint goal.";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "CHOMP planning failed.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "CHOMP planning succeeded.");

        auto future = std::async(std::launch::async, [this, my_plan]() {
            return this->move_group_interface_->execute(my_plan);
        });

        rclcpp::Rate loop_rate(10); 
        auto feedback = std::make_shared<GoToPose::Feedback>();

        while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            if (goal_handle->is_canceling()) {
                this->move_group_interface_->stop();
                result->success = false;
                result->message = "Action Canceled";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "GoToPose goal canceled");
                return;
            }

            feedback->current_pose = this->move_group_interface_->getCurrentPose();
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }

        moveit::core::MoveItErrorCode final_code = future.get();
        if (final_code == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true;
            result->message = "Goal reached successfully";
            goal_handle->succeed(result);
        } else {
            result->success = false;
            result->message = "Execution failed";
            goal_handle->abort(result);
        }
    }
};
} // namespace wrs25_arm_actions

RCLCPP_COMPONENTS_REGISTER_NODE(wrs25_arm_actions::GoToPoseActionServer)
