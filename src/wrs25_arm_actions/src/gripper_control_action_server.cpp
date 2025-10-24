#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

#include "wrs25_arm_actions/action/gripper_control.hpp"

namespace wrs25_arm_actions
{
class GripperControlActionServer : public rclcpp::Node
{
public:
    using GripperControl = wrs25_arm_actions::action::GripperControl;
    using GoalHandleGripperControl = rclcpp_action::ServerGoalHandle<GripperControl>;

    explicit GripperControlActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("gripper_control_action_server", options)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<GripperControl>(
            this,
            "gripper_control",
            std::bind(&GripperControlActionServer::handle_goal, this, _1, _2),
            std::bind(&GripperControlActionServer::handle_cancel, this, _1),
            std::bind(&GripperControlActionServer::handle_accepted, this, _1));

        this->publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/hand_controller/joint_trajectory", 10);

        this->joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&GripperControlActionServer::joint_state_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "GripperControlActionServer has been initialized.");
    }

private:
    rclcpp_action::Server<GripperControl>::SharedPtr action_server_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::string gripper_joint_name_ = "ur5_robotiq_85_left_knuckle_joint";
    double current_joint_position_ = 0.0;
    int gripper_joint_index_ = -1;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (gripper_joint_index_ < 0) {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == gripper_joint_name_) {
                    gripper_joint_index_ = i;
                    break;
                }
            }
        }

        if (gripper_joint_index_ >= 0) {
            current_joint_position_ = msg->position[gripper_joint_index_];
        }
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GripperControl::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received gripper goal request: position %f", goal->position);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGripperControl> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel gripper goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGripperControl> goal_handle)
    {
        std::thread{std::bind(&GripperControlActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleGripperControl> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<GripperControl::Feedback>();
        auto result = std::make_shared<GripperControl::Result>();

        trajectory_msgs::msg::JointTrajectory trajectory_msg;
        trajectory_msg.joint_names.push_back(gripper_joint_name_);
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.push_back(goal->position);
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        trajectory_msg.points.push_back(point);
        publisher_->publish(trajectory_msg);

        rclcpp::Rate loop_rate(20); // 20 Hz
        auto start_time = this->get_clock()->now();
        double timeout = 5.0; // 5 seconds

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Action Canceled";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Gripper goal canceled");
                return;
            }

            if ((this->get_clock()->now() - start_time).seconds() > timeout) {
                result->success = false;
                result->message = "Action timed out";
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "Gripper control timed out");
                return;
            }

            feedback->current_position = this->current_joint_position_;
            goal_handle->publish_feedback(feedback);

            double tolerance = 0.01; // Radians
            if (std::abs(this->current_joint_position_ - goal->position) < tolerance) {
                rclcpp::sleep_for(std::chrono::milliseconds(500)); // Wait half a second to stabilize
                feedback->current_position = this->current_joint_position_;
                if (std::abs(this->current_joint_position_ - goal->position) < tolerance) {
                    result->success = true;
                    result->message = "Goal reached";
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Gripper goal reached");
                    return;
                }
            }

            loop_rate.sleep();
        }
    }
};

} // namespace wrs25_arm_actions

RCLCPP_COMPONENTS_REGISTER_NODE(wrs25_arm_actions::GripperControlActionServer)
