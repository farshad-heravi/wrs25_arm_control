#include <memory>
#include <thread>
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
        : Node("go_to_pose_action_server", options),
          move_group_interface_(std::shared_ptr<rclcpp::Node>(this), "ur5_arm")
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
    moveit::planning_interface::MoveGroupInterface move_group_interface_;

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
        // Here you would add logic to stop the robot's movement
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        using namespace std::placeholders;
        // Execute the action in a separate thread to avoid blocking the main executor
        std::thread{std::bind(&GoToPoseActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<GoToPose::Result>();

        this->move_group_interface_.setPoseTarget(goal->target_pose.pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (this->move_group_interface_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        if(success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
            if(this->move_group_interface_.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS){
                result->success = true;
                result->message = "Goal reached successfully";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
            } else {
                result->success = false;
                result->message = "Execution failed";
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "Execution failed");
            }
        } else {
            result->success = false;
            result->message = "Planning failed";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
        }
    }
};
} // namespace wrs25_arm_actions

RCLCPP_COMPONENTS_REGISTER_NODE(wrs25_arm_actions::GoToPoseActionServer)
