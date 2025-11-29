#include <memory>
#include <thread>
#include <future>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include "wrs25_arm_actions/action/circ_motion.hpp"

namespace wrs25_arm_actions
{
class CircMotionActionServer : public rclcpp::Node
{
public:
    using CircMotion = wrs25_arm_actions::action::CircMotion;
    using GoalHandleCircMotion = rclcpp_action::ServerGoalHandle<CircMotion>;

    explicit CircMotionActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("circ_motion_action_server", options)
    {
        using namespace std::placeholders;

        // Initialize action server
        this->action_server_ = rclcpp_action::create_server<CircMotion>(
            this,
            "circ_motion",
            std::bind(&CircMotionActionServer::handle_goal, this, _1, _2),
            std::bind(&CircMotionActionServer::handle_cancel, this, _1),
            std::bind(&CircMotionActionServer::handle_accepted, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "CircMotionActionServer has been initialized and is awaiting goals.");
    }

private:
    rclcpp_action::Server<CircMotion>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CircMotion::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received circular motion goal request");
        RCLCPP_INFO(this->get_logger(), "Target: [%.3f, %.3f, %.3f], Center: [%.3f, %.3f, %.3f]",
                    goal->target_pose.pose.position.x,
                    goal->target_pose.pose.position.y,
                    goal->target_pose.pose.position.z,
                    goal->center_x, goal->center_y, goal->center_z);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleCircMotion> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel circular motion goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCircMotion> goal_handle)
    {
        using namespace std::placeholders;
        // Start a separate thread to execute the goal to avoid blocking the action server
        std::thread{std::bind(&CircMotionActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleCircMotion> goal_handle)
    {
        // 1. Setup MoveIt Interface (if not already done)
        if (!this->move_group_interface_) {
            auto node_ptr = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
            this->move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                node_ptr, "ur5_arm");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized for ur5_arm group.");
        }

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<CircMotion::Result>();

        RCLCPP_INFO(this->get_logger(), "Target pose: [%.3f, %.3f, %.3f]", 
                    goal->target_pose.pose.position.x, 
                    goal->target_pose.pose.position.y, 
                    goal->target_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Center point: [%.3f, %.3f, %.3f]",
                    goal->center_x, goal->center_y, goal->center_z);

        // 2. Clear previous targets to avoid multiple constraint errors
        this->move_group_interface_->clearPoseTargets();
        RCLCPP_INFO(this->get_logger(), "Cleared previous pose targets");

        // 3. Set TCP Link (End Effector)
        std::string tcp_link = goal->tcp_link.empty() ? "ur5_gripper_tcp" : goal->tcp_link;
        this->move_group_interface_->setEndEffectorLink(tcp_link);
        RCLCPP_INFO(this->get_logger(), "Set end effector link to: %s", tcp_link.c_str());

        // 4. Set Pilz CIRC Planner
        this->move_group_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
        this->move_group_interface_->setPlannerId("CIRC");
        RCLCPP_INFO(this->get_logger(), "Set planner to Pilz CIRC");

        // 4.1 set velocity and acceleration scaling factors
        double velocity_scaling = (goal->velocity_scaling > 0.0 && goal->velocity_scaling <= 1.0) 
                                  ? goal->velocity_scaling : 0.1;
        double acceleration_scaling = (goal->acceleration_scaling > 0.0 && goal->acceleration_scaling <= 1.0) 
                                      ? goal->acceleration_scaling : 0.1;
        this->move_group_interface_->setMaxVelocityScalingFactor(velocity_scaling);
        this->move_group_interface_->setMaxAccelerationScalingFactor(acceleration_scaling);
        RCLCPP_INFO(this->get_logger(), "Set velocity scaling: %.3f, acceleration scaling: %.3f", 
                    velocity_scaling, acceleration_scaling);

        // 5. Use the geometric CENTER as the constraint point with a large tolerance region
        // User provides the CENTER of the circle (pivot point)
        // With a large tolerance sphere (2× radius) centered at the geometric center,
        // the entire circular arc is contained within the constraint region
        
        // Get current pose as starting point
        geometry_msgs::msg::PoseStamped current_pose = this->move_group_interface_->getCurrentPose(tcp_link);
        
        double start_x = current_pose.pose.position.x;
        double start_y = current_pose.pose.position.y;
        double start_z = current_pose.pose.position.z;
        
        double target_x = goal->target_pose.pose.position.x;
        double target_y = goal->target_pose.pose.position.y;
        double target_z = goal->target_pose.pose.position.z;
        
        RCLCPP_INFO(this->get_logger(), "Start pose: [%.4f, %.4f, %.4f]", start_x, start_y, start_z);
        RCLCPP_INFO(this->get_logger(), "Target pose: [%.4f, %.4f, %.4f]", target_x, target_y, target_z);
        RCLCPP_INFO(this->get_logger(), "Arc center: [%.4f, %.4f, %.4f]", 
                    goal->center_x, goal->center_y, goal->center_z);
        
        // Calculate vectors from center to start and target
        double vec_start_x = start_x - goal->center_x;
        double vec_start_y = start_y - goal->center_y;
        double vec_start_z = start_z - goal->center_z;
        
        double vec_target_x = target_x - goal->center_x;
        double vec_target_y = target_y - goal->center_y;
        double vec_target_z = target_z - goal->center_z;
        
        // Calculate radii to verify they're equal (or close)
        double radius_start = std::sqrt(vec_start_x*vec_start_x + vec_start_y*vec_start_y + vec_start_z*vec_start_z);
        double radius_target = std::sqrt(vec_target_x*vec_target_x + vec_target_y*vec_target_y + vec_target_z*vec_target_z);
        
        RCLCPP_INFO(this->get_logger(), "Radius from center to start: %.4f", radius_start);
        RCLCPP_INFO(this->get_logger(), "Radius from center to target: %.4f", radius_target);
        
        if (std::abs(radius_start - radius_target) > 0.01) {
            RCLCPP_WARN(this->get_logger(), 
                       "Start and target are not equidistant from center (difference: %.4fm). Arc may be imprecise.",
                       std::abs(radius_start - radius_target));
        }
        
        // Use average radius for the constraint tolerance
        double avg_radius = (radius_start + radius_target) / 2.0;
        
        RCLCPP_INFO(this->get_logger(), "Average arc radius: %.4f m", avg_radius);
        
        // Create path constraint centered at the geometric center of the arc
        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.name = "center";  // Pilz CIRC accepts "center" or "interim"
        
        // Add position constraint for the center point
        moveit_msgs::msg::PositionConstraint center_constraint;
        center_constraint.header.frame_id = goal->target_pose.header.frame_id;
        center_constraint.link_name = tcp_link;
        
        // Define a large tolerance sphere centered at the geometric center
        // The sphere must contain the entire circular arc (radius = 2× arc radius)
        // This satisfies both: (1) Pilz CIRC planning, (2) MoveIt trajectory validation
        shape_msgs::msg::SolidPrimitive sphere;
        sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        sphere.dimensions.resize(1);
        sphere.dimensions[0] = avg_radius * 2.0;  // 2× radius ensures entire arc is within constraint
        
        geometry_msgs::msg::Pose center_pose;
        center_pose.position.x = goal->center_x;
        center_pose.position.y = goal->center_y;
        center_pose.position.z = goal->center_z;
        center_pose.orientation.w = 1.0;  // Identity orientation for position constraint
        
        center_constraint.constraint_region.primitives.push_back(sphere);
        center_constraint.constraint_region.primitive_poses.push_back(center_pose);
        center_constraint.weight = 1.0;
        
        path_constraints.position_constraints.push_back(center_constraint);

        // Set the path constraints
        this->move_group_interface_->setPathConstraints(path_constraints);
        
        RCLCPP_INFO(this->get_logger(), "Set CIRC path constraint: center at [%.4f, %.4f, %.4f] with tolerance radius %.4f m",
                    goal->center_x, goal->center_y, goal->center_z, sphere.dimensions[0]);

        // 6. Set target pose and plan with CIRC
        this->move_group_interface_->setPoseTarget(goal->target_pose);
        
        // Create motion plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        // Attempt planning with CIRC planner
        RCLCPP_INFO(this->get_logger(), "Planning circular motion from current pose to target through interim point");
        
        // Disable path constraint checking during state validity
        // The Pilz CIRC planner uses "interim"/"center" constraint ONLY for determining the arc shape,
        // NOT for validating every waypoint along the trajectory
        this->move_group_interface_->setStartStateToCurrentState();
        
        moveit::core::MoveItErrorCode plan_result = this->move_group_interface_->plan(my_plan);
        
        // Clear path constraints immediately after planning
        this->move_group_interface_->clearPathConstraints();
        
        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = false;
            result->message = "CIRC planning failed for circular motion.";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "CIRC planning failed for target pose.");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Circular motion planning succeeded. Trajectory has %zu waypoints.",
                    my_plan.trajectory_.joint_trajectory.points.size());

        // 7. Execution and Feedback Loop
        auto future = std::async(std::launch::async, [this, my_plan]() {
            return this->move_group_interface_->execute(my_plan);
        });

        rclcpp::Rate loop_rate(10); 
        auto feedback = std::make_shared<CircMotion::Feedback>();

        while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            if (goal_handle->is_canceling()) {
                this->move_group_interface_->stop();
                result->success = false;
                result->message = "Circular motion canceled by user request.";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "CircMotion goal canceled.");
                return;
            }

            feedback->current_pose = this->move_group_interface_->getCurrentPose();
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }

        // 8. Final Result
        moveit::core::MoveItErrorCode final_code = future.get();
        if (final_code == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true;
            result->message = "Circular motion completed successfully.";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Circular motion execution succeeded.");
        } else {
            result->success = false;
            result->message = "Circular motion execution failed with code: " + std::to_string(final_code.val);
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Circular motion execution failed.");
        }
    }
};
} // namespace wrs25_arm_actions

RCLCPP_COMPONENTS_REGISTER_NODE(wrs25_arm_actions::CircMotionActionServer)

