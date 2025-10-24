#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include "wrs25_arm_actions/action/go_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GoToPoseActionClient : public rclcpp::Node
{
public:
    using GoToPose = wrs25_arm_actions::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;

    explicit GoToPoseActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
        : Node("go_to_pose_action_client", node_options)
    {
        this->client_ptr_ = rclcpp_action::create_client<GoToPose>(this, "go_to_pose");
    }

    void send_goal(const geometry_msgs::msg::PoseStamped & target_pose)
    {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = GoToPose::Goal();
        goal_msg.target_pose = target_pose;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<GoToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&GoToPoseActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = 
            std::bind(&GoToPoseActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&GoToPoseActionClient::result_callback, this, std::placeholders::_1);
        
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<GoToPose>::SharedPtr client_ptr_;

    void goal_response_callback(const GoalHandleGoToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleGoToPose::SharedPtr,
        const std::shared_ptr<const GoToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Feedback received: Current pose is approximately at z = %f", feedback->current_pose.pose.position.z);
    }

    void result_callback(const GoalHandleGoToPose::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
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
        RCLCPP_INFO(this->get_logger(), "Result received: %s", result.result->message.c_str());
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Create a temporary node to declare and get parameters
    auto param_client_node = std::make_shared<GoToPoseActionClient>();

    param_client_node->declare_parameter<double>("posx", 0.0);
    param_client_node->declare_parameter<double>("posy", 0.0);
    param_client_node->declare_parameter<double>("posz", 0.0);
    param_client_node->declare_parameter<double>("orw", 1.0);
    param_client_node->declare_parameter<double>("orx", 0.0);
    param_client_node->declare_parameter<double>("ory", 0.0);
    param_client_node->declare_parameter<double>("orz", 0.0);

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "world";
    goal_pose.pose.position.x = param_client_node->get_parameter("posx").as_double();
    goal_pose.pose.position.y = param_client_node->get_parameter("posy").as_double();
    goal_pose.pose.position.z = param_client_node->get_parameter("posz").as_double();
    goal_pose.pose.orientation.w = param_client_node->get_parameter("orw").as_double();
    goal_pose.pose.orientation.x = param_client_node->get_parameter("orx").as_double();
    goal_pose.pose.orientation.y = param_client_node->get_parameter("ory").as_double();
    goal_pose.pose.orientation.z = param_client_node->get_parameter("orz").as_double();

    param_client_node->send_goal(goal_pose);

    rclcpp::spin(param_client_node);
    rclcpp::shutdown();
    return 0;
}
