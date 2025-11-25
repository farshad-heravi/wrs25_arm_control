#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include "wrs25_arm_actions/action/robotiq_gripper_control.hpp"

class RobotiqGripperActionClient : public rclcpp::Node
{
public:
    using RobotiqGripperControl = wrs25_arm_actions::action::RobotiqGripperControl;
    using GoalHandleRobotiqGripper = rclcpp_action::ClientGoalHandle<RobotiqGripperControl>;

    explicit RobotiqGripperActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
        : Node("robotiq_gripper_action_client", node_options)
    {
        this->client_ptr_ = rclcpp_action::create_client<RobotiqGripperControl>(
            this, "robotiq_gripper_control");
    }

    /**
     * @brief Sends the RobotiqGripperControl goal to the action server.
     * @param command The gripper command ("open", "close", or "move").
     * @param position Target position (0-255) for "move" command.
     * @param speed Movement speed (0-255).
     * @param force Gripping force (0-255).
     * @param ip_address Optional gripper IP address.
     * @param port Optional gripper port.
     */
    void send_goal(const std::string& command,
                   int32_t position = 0,
                   int32_t speed = 255,
                   int32_t force = 255,
                   const std::string& ip_address = "",
                   int32_t port = 0)
    {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = RobotiqGripperControl::Goal();
        goal_msg.command = command;
        goal_msg.position = position;
        goal_msg.speed = speed;
        goal_msg.force = force;
        goal_msg.ip_address = ip_address;
        goal_msg.port = port;

        RCLCPP_INFO(this->get_logger(), 
                    "Sending goal - Command: '%s', Position: %d, Speed: %d, Force: %d",
                    command.c_str(), position, speed, force);

        auto send_goal_options = rclcpp_action::Client<RobotiqGripperControl>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&RobotiqGripperActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = 
            std::bind(&RobotiqGripperActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&RobotiqGripperActionClient::result_callback, this, std::placeholders::_1);
        
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<RobotiqGripperControl>::SharedPtr client_ptr_;

    void goal_response_callback(const GoalHandleRobotiqGripper::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleRobotiqGripper::SharedPtr,
        const std::shared_ptr<const RobotiqGripperControl::Feedback> feedback)
    {
        RCLCPP_INFO(
            this->get_logger(), 
            "Feedback - Position: %d, Moving: %s", 
            feedback->current_position,
            feedback->is_moving ? "Yes" : "No");
    }

    void result_callback(const GoalHandleRobotiqGripper::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal SUCCEEDED!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was ABORTED");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was CANCELED");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        
        if (result.result->success) {
            RCLCPP_INFO(this->get_logger(), 
                       "Result - Success: %s, Final Position: %d, Message: %s",
                       result.result->success ? "true" : "false",
                       result.result->final_position,
                       result.result->message.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                        "Result - Success: false, Message: %s",
                        result.result->message.c_str());
        }
        
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto client_node = std::make_shared<RobotiqGripperActionClient>();

    // Declare parameters
    client_node->declare_parameter<std::string>("command", "open");
    client_node->declare_parameter<int>("position", 0);
    client_node->declare_parameter<int>("speed", 255);
    client_node->declare_parameter<int>("force", 255);
    client_node->declare_parameter<std::string>("ip_address", "");
    client_node->declare_parameter<int>("port", 0);

    // Get parameters
    std::string command = client_node->get_parameter("command").as_string();
    int32_t position = client_node->get_parameter("position").as_int();
    int32_t speed = client_node->get_parameter("speed").as_int();
    int32_t force = client_node->get_parameter("force").as_int();
    std::string ip_address = client_node->get_parameter("ip_address").as_string();
    int32_t port = client_node->get_parameter("port").as_int();

    // Send goal
    client_node->send_goal(command, position, speed, force, ip_address, port);

    rclcpp::spin(client_node);
    rclcpp::shutdown();
    return 0;
}











