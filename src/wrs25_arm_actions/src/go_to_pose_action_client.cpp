#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <thread>
#include "wrs25_arm_actions/action/go_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class GoToPoseActionClient : public rclcpp::Node
{
public:
    using GoToPose = wrs25_arm_actions::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;

    explicit GoToPoseActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
        : Node("go_to_pose_action_client", node_options)
    {
        this->client_ptr_ = rclcpp_action::create_client<GoToPose>(this, "go_to_pose");

        // add publisher for target pose marker
        rclcpp::QoS qos(10);
        qos.transient_local();
        qos.reliable();
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("target_pose_marker", qos);
    }

    /**
     * @brief Sends the GoToPose goal to the action server with specified planner configurations.
     * @param target_pose The desired target pose.
     * @param pipeline_id The ID of the planning pipeline (e.g., "ompl", "pilz_industrial_motion_planner").
     * @param planner_id The ID of the specific planner within the pipeline (e.g., "RRTkConfigDefault", "LIN").
     */
    void send_goal(const geometry_msgs::msg::PoseStamped & target_pose, 
                   const std::string& pipeline_id, 
                   const std::string& planner_id)
    {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            // Do not shutdown here, allow the main loop to handle shutdown
            return;
        }

        auto goal_msg = GoToPose::Goal();
        goal_msg.target_pose = target_pose;
        goal_msg.planning_pipeline_id = pipeline_id;
        goal_msg.planner_id = planner_id;

        RCLCPP_INFO(this->get_logger(), "Sending goal with Pipeline: '%s', Planner: '%s'",
                    pipeline_id.c_str(), planner_id.c_str());

        auto send_goal_options = rclcpp_action::Client<GoToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&GoToPoseActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = 
            std::bind(&GoToPoseActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&GoToPoseActionClient::result_callback, this, std::placeholders::_1);
        
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        publish_marker(target_pose);
    }

private:
    std::string target_frame_id = "ur5_base_link";

    rclcpp_action::Client<GoToPose>::SharedPtr client_ptr_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    void publish_marker(const geometry_msgs::msg::PoseStamped & pose)
    {
        std::array<std::array<float, 3>, 3> axis_colors = {{
            {1.0, 0.0, 0.0},  // X = Red
            {0.0, 1.0, 0.0},  // Y = Green
            {0.0, 0.0, 1.0}   // Z = Blue
        }};
        
        // Create quaternion from pose orientation
        tf2::Quaternion quat(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        );
        
        // Convert quaternion to rotation matrix
        tf2::Matrix3x3 rotation_matrix(quat);
        
        // Define base axis directions (in local frame)
        std::array<tf2::Vector3, 3> base_axes = {
            tf2::Vector3(1.0, 0.0, 0.0),  // X
            tf2::Vector3(0.0, 1.0, 0.0),  // Y
            tf2::Vector3(0.0, 0.0, 1.0)   // Z
        };
        
        // Rotate axes according to pose orientation
        std::array<tf2::Vector3, 3> rotated_axes;
        for (size_t i = 0; i < 3; i++)
        {
            rotated_axes[i] = rotation_matrix * base_axes[i];
        }

        // Publish markers briefly to ensure Rviz gets the message
        for (size_t j = 0; j < 5; j++)
        {
            for (size_t i = 0; i < 3; i++)
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = target_frame_id;
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "target_pose";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::ARROW;
                marker.action = visualization_msgs::msg::Marker::ADD;

                // Start at pose position, end along rotated axis direction
                marker.points.resize(2);
                marker.points[0] = pose.pose.position;
                marker.points[1].x = pose.pose.position.x + rotated_axes[i].x() * 0.2;
                marker.points[1].y = pose.pose.position.y + rotated_axes[i].y() * 0.2;
                marker.points[1].z = pose.pose.position.z + rotated_axes[i].z() * 0.2;

                marker.scale.x = 0.01;  // shaft diameter
                marker.scale.y = 0.02;  // head diameter
                marker.scale.z = 0.03;  // head length

                marker.color.r = axis_colors[i][0];
                marker.color.g = axis_colors[i][1];
                marker.color.b = axis_colors[i][2];
                marker.color.a = 1.0;

                // Marker is permanent (until node shutdown)
                marker.lifetime = rclcpp::Duration::from_seconds(0);
                
                marker_pub_->publish(marker);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

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
        RCLCPP_INFO(
            this->get_logger(), 
            "Feedback received: Current pose: [x: %.2f, y: %.2f, z: %.2f]", 
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y,
            feedback->current_pose.pose.position.z);
    }

    void result_callback(const GoalHandleGoToPose::WrappedResult & result)
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
        RCLCPP_INFO(this->get_logger(), "Result received: %s", result.result->message.c_str());
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Create a temporary node to declare and get parameters
    auto client_node = std::make_shared<GoToPoseActionClient>();

    // 1. Declare and retrieve Pose parameters (Existing)
    client_node->declare_parameter<double>("posx", 0.0);
    client_node->declare_parameter<double>("posy", 0.0);
    client_node->declare_parameter<double>("posz", 0.0);
    client_node->declare_parameter<double>("orw", 1.0);
    client_node->declare_parameter<double>("orx", 0.0);
    client_node->declare_parameter<double>("ory", 0.0);
    client_node->declare_parameter<double>("orz", 0.0);

    // 2. Declare and retrieve Planner parameters (New)
    // Defaulting to empty strings lets the action server use MoveIt's defaults.
    client_node->declare_parameter<std::string>("pipeline", "");
    client_node->declare_parameter<std::string>("planner", "");

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "ur5_base_link";
    goal_pose.pose.position.x = client_node->get_parameter("posx").as_double();
    goal_pose.pose.position.y = client_node->get_parameter("posy").as_double();
    goal_pose.pose.position.z = client_node->get_parameter("posz").as_double();
    goal_pose.pose.orientation.w = client_node->get_parameter("orw").as_double();
    goal_pose.pose.orientation.x = client_node->get_parameter("orx").as_double();
    goal_pose.pose.orientation.y = client_node->get_parameter("ory").as_double();
    goal_pose.pose.orientation.z = client_node->get_parameter("orz").as_double();
    
    std::string pipeline_id = client_node->get_parameter("pipeline").as_string();
    std::string planner_id = client_node->get_parameter("planner").as_string();

    // 3. Send goal with all parameters
    client_node->send_goal(goal_pose, pipeline_id, planner_id);

    rclcpp::spin(client_node);
    rclcpp::shutdown();
    return 0;
}
