#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>

#include "wrs25_arm_actions/action/gripper_control.hpp"

class GripperControlActionClient : public rclcpp::Node
{
public:
    using GripperControl = wrs25_arm_actions::action::GripperControl;
    using GoalHandleGripperControl = rclcpp_action::ClientGoalHandle<GripperControl>;

    explicit GripperControlActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
        : Node("gripper_control_action_client", node_options)
    {
        this->client_ptr_ = rclcpp_action::create_client<GripperControl>(this, "gripper_control");
    }

    void send_goal(double position)
    {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = GripperControl::Goal();
        goal_msg.position = position;

        RCLCPP_INFO(this->get_logger(), "Sending gripper goal: %f", position);

        auto send_goal_options = rclcpp_action::Client<GripperControl>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&GripperControlActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = 
            std::bind(&GripperControlActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&GripperControlActionClient::result_callback, this, std::placeholders::_1);
        
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<GripperControl>::SharedPtr client_ptr_;

    void goal_response_callback(const GoalHandleGripperControl::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleGripperControl::SharedPtr,
        const std::shared_ptr<const GripperControl::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Feedback received: current position = %f", feedback->current_position);
    }

    void result_callback(const GoalHandleGripperControl::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto action_client = std::make_shared<GripperControlActionClient>();

    action_client->declare_parameter<double>("position", 0.0);
    double goal_position = action_client->get_parameter("position").as_double();

    action_client->send_goal(goal_position);

    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}
