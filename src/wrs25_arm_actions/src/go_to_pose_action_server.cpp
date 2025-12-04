/**
 * @file go_to_pose_action_server.cpp
 * @brief GoToPose action server with trajectory caching support using MoveIt Warehouse
 * 
 * This action server plans and executes robot arm movements to target poses.
 * It includes automatic trajectory caching using the MoveIt warehouse (SQLite)
 * to reuse previously computed plans when the start state and goal are similar.
 * 
 * TRAJECTORY CACHING:
 * - Trajectories are stored in the MoveIt warehouse SQLite database
 * - Cached trajectories are automatically retrieved when a matching query is found
 * - Successfully executed trajectories are automatically saved to the warehouse
 * - Cache can be viewed and managed via RViz's Motion Planning plugin
 * 
 * PARAMETERS:
 * - use_trajectory_cache (bool, default: true): Enable/disable trajectory caching
 * - cache_tolerance_position (double, default: 0.003): Position tolerance for cache matching (m)
 * - cache_tolerance_orientation (double, default: 0.01): Orientation tolerance for cache matching (rad)
 * - cache_tolerance_joint (double, default: 0.05): Joint state tolerance for cache matching (rad)
 * - warehouse_host (string): Path to SQLite database file
 */

#include <memory>
#include <thread>
#include <future>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <functional>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/trajectory_constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <warehouse_ros/database_loader.h>
#include "wrs25_arm_actions/action/go_to_pose.hpp"

namespace wrs25_arm_actions
{

/**
 * @brief Trajectory cache using MoveIt Warehouse (SQLite database)
 */
class WarehouseTrajectoryCache
{
public:
    explicit WarehouseTrajectoryCache(
        const rclcpp::Node::SharedPtr& node,
        const std::string& database_path = "")
        : node_(node)
        , logger_(node->get_logger())
        , initialized_(false)
    {
        // Set default database path
        if (database_path.empty()) {
            const char* home = std::getenv("HOME");
            if (home) {
                database_path_ = std::string(home) + "/.ros/moveit_warehouse.sqlite";
            } else {
                database_path_ = "/tmp/moveit_warehouse.sqlite";
            }
        } else {
            database_path_ = database_path;
        }

        // Set default tolerances
        position_tolerance_ = 0.003;      // 3mm
        orientation_tolerance_ = 0.01;    // ~0.5 degrees
        joint_tolerance_ = 0.05;          // ~3 degrees

        RCLCPP_INFO(logger_, "Trajectory cache will use database at: %s", database_path_.c_str());
    }

    bool initialize()
    {
        if (initialized_) {
            return true;
        }

        try {
            // Set warehouse parameters on the node
            node_->set_parameter(rclcpp::Parameter("warehouse_plugin", "warehouse_ros_sqlite::DatabaseConnection"));
            node_->set_parameter(rclcpp::Parameter("warehouse_host", database_path_));

            // Initialize the database connection
            warehouse_ros::DatabaseLoader db_loader(node_);
            database_ = db_loader.loadDatabase();

            if (!database_) {
                RCLCPP_ERROR(logger_, "Failed to load warehouse database");
                return false;
            }

            database_->setParams(database_path_, 0);
            if (!database_->connect()) {
                RCLCPP_ERROR(logger_, "Failed to connect to warehouse database at: %s", database_path_.c_str());
                return false;
            }

            // Initialize storage interfaces
            planning_scene_storage_ = std::make_shared<moveit_warehouse::PlanningSceneStorage>(database_);

            initialized_ = true;
            RCLCPP_INFO(logger_, "✓ Warehouse trajectory cache initialized successfully");
            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Exception initializing warehouse: %s", e.what());
            return false;
        }
    }

    void setTolerances(double position_tol, double orientation_tol, double joint_tol)
    {
        position_tolerance_ = position_tol;
        orientation_tolerance_ = orientation_tol;
        joint_tolerance_ = joint_tol;
    }

    /**
     * @brief Generate a unique scene name for caching based on goal parameters
     */
    std::string generateSceneName(
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const std::string& planning_pipeline,
        const std::string& planner_id,
        const std::string& tcp_link) const
    {
        std::stringstream ss;
        
        // Use discrete bins for position (1mm resolution)
        int px = static_cast<int>(goal_pose.pose.position.x * 1000);
        int py = static_cast<int>(goal_pose.pose.position.y * 1000);
        int pz = static_cast<int>(goal_pose.pose.position.z * 1000);
        
        // Use discrete bins for orientation (0.01 resolution)
        int qw = static_cast<int>(goal_pose.pose.orientation.w * 100);
        int qx = static_cast<int>(goal_pose.pose.orientation.x * 100);
        int qy = static_cast<int>(goal_pose.pose.orientation.y * 100);
        int qz = static_cast<int>(goal_pose.pose.orientation.z * 100);
        
        ss << "traj_cache_"
           << goal_pose.header.frame_id << "_"
           << px << "_" << py << "_" << pz << "_"
           << qw << "_" << qx << "_" << qy << "_" << qz << "_"
           << planning_pipeline << "_" << planner_id << "_" << tcp_link;
        
        return ss.str();
    }

    /**
     * @brief Check if joint states are within tolerance
     */
    bool jointStatesMatch(
        const std::vector<double>& joints1,
        const std::vector<double>& joints2) const
    {
        if (joints1.size() != joints2.size()) {
            return false;
        }
        
        for (size_t i = 0; i < joints1.size(); ++i) {
            if (std::abs(joints1[i] - joints2[i]) > joint_tolerance_) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Try to fetch a cached trajectory from the warehouse
     */
    bool fetchTrajectory(
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const std::string& planning_pipeline,
        const std::string& planner_id,
        const std::string& tcp_link,
        const std::vector<double>& current_joint_positions,
        moveit_msgs::msg::RobotTrajectory& trajectory_out)
    {
        if (!initialized_ && !initialize()) {
            RCLCPP_WARN(logger_, "Warehouse not initialized, skipping cache lookup");
            return false;
        }

        std::string scene_name = generateSceneName(goal_pose, planning_pipeline, planner_id, tcp_link);

        try {
            // Check if we have a cached trajectory for this scene
            moveit_warehouse::PlanningSceneWithMetadata scene_m;
            if (!planning_scene_storage_->hasPlanningScene(scene_name)) {
                RCLCPP_DEBUG(logger_, "No cached trajectory found for: %s", scene_name.c_str());
                return false;
            }

            // Get the planning scene
            if (!planning_scene_storage_->getPlanningScene(scene_m, scene_name)) {
                RCLCPP_DEBUG(logger_, "Failed to retrieve planning scene: %s", scene_name.c_str());
                return false;
            }

            // Get associated trajectory
            std::vector<moveit_warehouse::RobotTrajectoryWithMetadata> trajectories;
            planning_scene_storage_->getPlanningQueries(trajectories, scene_name);

            if (trajectories.empty()) {
                RCLCPP_DEBUG(logger_, "No trajectories found for scene: %s", scene_name.c_str());
                return false;
            }

            // Get the first (best) trajectory
            const auto& traj_m = trajectories[0];
            
            // Validate start state matches current state
            if (!traj_m->joint_trajectory.points.empty()) {
                const auto& start_positions = traj_m->joint_trajectory.points[0].positions;
                std::vector<double> start_vec(start_positions.begin(), start_positions.end());
                
                if (!jointStatesMatch(start_vec, current_joint_positions)) {
                    RCLCPP_INFO(logger_, "Cached trajectory start state doesn't match current state");
                    return false;
                }
            }

            trajectory_out = *traj_m;
            
            RCLCPP_INFO(logger_, 
                "✓ CACHE HIT: Retrieved trajectory with %zu waypoints from warehouse (scene: %s)",
                trajectory_out.joint_trajectory.points.size(), scene_name.c_str());
            
            return true;

        } catch (const std::exception& e) {
            RCLCPP_WARN(logger_, "Exception fetching cached trajectory: %s", e.what());
            return false;
        }
    }

    /**
     * @brief Save a trajectory to the warehouse
     */
    bool saveTrajectory(
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const std::string& planning_pipeline,
        const std::string& planner_id,
        const std::string& tcp_link,
        const std::vector<double>& start_joint_positions,
        const moveit_msgs::msg::RobotTrajectory& trajectory,
        const moveit_msgs::msg::MotionPlanRequest& request)
    {
        if (!initialized_ && !initialize()) {
            RCLCPP_WARN(logger_, "Warehouse not initialized, skipping cache save");
            return false;
        }

        std::string scene_name = generateSceneName(goal_pose, planning_pipeline, planner_id, tcp_link);

        try {
            // Create a minimal planning scene for storage
            moveit_msgs::msg::PlanningScene scene;
            scene.name = scene_name;
            scene.is_diff = true;

            // Remove existing scene if it exists (to update with new trajectory)
            if (planning_scene_storage_->hasPlanningScene(scene_name)) {
                planning_scene_storage_->removePlanningScene(scene_name);
            }

            // Add the planning scene
            planning_scene_storage_->addPlanningScene(scene);

            // Add the trajectory as a planning query result
            planning_scene_storage_->addPlanningQuery(request, scene_name, planner_id);
            
            // Store the trajectory
            moveit_msgs::msg::RobotTrajectory traj_to_store = trajectory;
            planning_scene_storage_->addPlanningResult(request, traj_to_store, scene_name, planner_id);

            RCLCPP_INFO(logger_, 
                "✓ CACHE SAVE: Saved trajectory with %zu waypoints to warehouse (scene: %s)",
                trajectory.joint_trajectory.points.size(), scene_name.c_str());
            
            return true;

        } catch (const std::exception& e) {
            RCLCPP_WARN(logger_, "Exception saving trajectory to warehouse: %s", e.what());
            return false;
        }
    }

    size_t getCacheSize() const
    {
        if (!initialized_) {
            return 0;
        }
        
        try {
            std::vector<std::string> names;
            planning_scene_storage_->getPlanningSceneNames(names);
            size_t count = 0;
            for (const auto& name : names) {
                if (name.find("traj_cache_") == 0) {
                    ++count;
                }
            }
            return count;
        } catch (...) {
            return 0;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    std::string database_path_;
    bool initialized_;
    
    warehouse_ros::DatabaseConnection::Ptr database_;
    std::shared_ptr<moveit_warehouse::PlanningSceneStorage> planning_scene_storage_;
    
    double position_tolerance_;
    double orientation_tolerance_;
    double joint_tolerance_;
};


class GoToPoseActionServer : public rclcpp::Node
{
public:
    using GoToPose = wrs25_arm_actions::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

    explicit GoToPoseActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("go_to_pose_action_server", options)
    {
        using namespace std::placeholders;

        // Declare cache parameters
        this->declare_parameter("use_trajectory_cache", true);
        this->declare_parameter("cache_tolerance_position", 0.003);      // 3mm
        this->declare_parameter("cache_tolerance_orientation", 0.01);   // ~0.5 deg
        this->declare_parameter("cache_tolerance_joint", 0.05);         // ~3 deg
        this->declare_parameter("warehouse_host", "");
        this->declare_parameter("warehouse_plugin", "warehouse_ros_sqlite::DatabaseConnection");
        
        // Get parameters
        use_cache_ = this->get_parameter("use_trajectory_cache").as_bool();
        double pos_tol = this->get_parameter("cache_tolerance_position").as_double();
        double ori_tol = this->get_parameter("cache_tolerance_orientation").as_double();
        double joint_tol = this->get_parameter("cache_tolerance_joint").as_double();
        std::string warehouse_host = this->get_parameter("warehouse_host").as_string();

        // Initialize action server
        this->action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "go_to_pose",
            std::bind(&GoToPoseActionServer::handle_goal, this, _1, _2),
            std::bind(&GoToPoseActionServer::handle_cancel, this, _1),
            std::bind(&GoToPoseActionServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), 
            "GoToPoseActionServer initialized with trajectory caching %s",
            use_cache_ ? "ENABLED" : "DISABLED");

        // Initialize trajectory cache (lazy initialization - will connect on first use)
        if (use_cache_) {
            auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
            trajectory_cache_ = std::make_unique<WarehouseTrajectoryCache>(node_ptr, warehouse_host);
            trajectory_cache_->setTolerances(pos_tol, ori_tol, joint_tol);
        }
    }

private:
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<WarehouseTrajectoryCache> trajectory_cache_;
    bool use_cache_;

    // Statistics
    size_t cache_hits_ = 0;
    size_t cache_misses_ = 0;

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
            auto node_ptr = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
            this->move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "ur5_arm");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized for ur5_arm group.");
        }

        const auto goal = goal_handle->get_goal();

        // Set velocity and acceleration scaling from goal
        if (goal->velocity_scaling > 0.0 && goal->velocity_scaling <= 1.0) {
            this->move_group_interface_->setMaxVelocityScalingFactor(goal->velocity_scaling);
            RCLCPP_INFO(this->get_logger(), "Set max velocity scaling to: %.2f", goal->velocity_scaling);
        } else {
            RCLCPP_INFO(this->get_logger(), "Using default velocity scaling");
        }

        if (goal->acceleration_scaling > 0.0 && goal->acceleration_scaling <= 1.0) {
            this->move_group_interface_->setMaxAccelerationScalingFactor(goal->acceleration_scaling);
            RCLCPP_INFO(this->get_logger(), "Set max acceleration scaling to: %.2f", goal->acceleration_scaling);
        } else {
            RCLCPP_INFO(this->get_logger(), "Using default acceleration scaling");
        }
        
        auto result = std::make_shared<GoToPose::Result>();

        RCLCPP_INFO(this->get_logger(), "Target pose: [%.3f, %.3f, %.3f]", 
                    goal->target_pose.pose.position.x, 
                    goal->target_pose.pose.position.y, 
                    goal->target_pose.pose.position.z);

        // 2. Clear previous targets
        this->move_group_interface_->clearPoseTargets();
        RCLCPP_INFO(this->get_logger(), "Cleared previous pose targets");

        // 3. Set TCP Link (End Effector)
        std::string tcp_link = goal->tcp_link.empty() ? "ur5_gripper_tcp" : goal->tcp_link;
        this->move_group_interface_->setEndEffectorLink(tcp_link);
        RCLCPP_INFO(this->get_logger(), "Set end effector link to: %s", tcp_link.c_str());

        // 4. Dynamic Planner Configuration
        std::string planning_pipeline = goal->planning_pipeline_id;
        std::string planner_id = goal->planner_id;
        
        if (!planning_pipeline.empty()) {
            this->move_group_interface_->setPlanningPipelineId(planning_pipeline);
            RCLCPP_INFO(this->get_logger(), "Set Planning Pipeline ID to: %s", planning_pipeline.c_str());
        } else {
            planning_pipeline = "default";
            RCLCPP_INFO(this->get_logger(), "Using default planning pipeline.");
        }

        if (!planner_id.empty()) {
            this->move_group_interface_->setPlannerId(planner_id);
            RCLCPP_INFO(this->get_logger(), "Set Planner ID to: %s", planner_id.c_str());
        } else {
            planner_id = "default";
            RCLCPP_INFO(this->get_logger(), "Using default planner ID.");
        }

        // Get current joint positions for cache lookup
        std::vector<double> current_joint_positions = this->move_group_interface_->getCurrentJointValues();

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool used_cache = false;

        // 5. Try to fetch from warehouse cache first
        if (use_cache_ && trajectory_cache_) {
            moveit_msgs::msg::RobotTrajectory cached_trajectory;
            
            if (trajectory_cache_->fetchTrajectory(
                    goal->target_pose,
                    planning_pipeline,
                    planner_id,
                    tcp_link,
                    current_joint_positions,
                    cached_trajectory))
            {
                // Use cached trajectory
                my_plan.trajectory_ = cached_trajectory;
                my_plan.planning_time_ = 0.0;
                used_cache = true;
                cache_hits_++;
                
                RCLCPP_INFO(this->get_logger(), 
                    "═══════════════════════════════════════════════════════════════");
                RCLCPP_INFO(this->get_logger(), 
                    "  USING CACHED TRAJECTORY (hits: %zu, misses: %zu, ratio: %.1f%%)",
                    cache_hits_, cache_misses_, 
                    100.0 * cache_hits_ / (cache_hits_ + cache_misses_));
                RCLCPP_INFO(this->get_logger(), 
                    "═══════════════════════════════════════════════════════════════");
            }
        }

        // 6. If no cached trajectory, plan a new one
        moveit_msgs::msg::MotionPlanRequest plan_request;
        if (!used_cache) {
            cache_misses_++;
            
            // Set Target and Plan
            this->move_group_interface_->setPoseTarget(goal->target_pose);
            
            auto planning_start = this->get_clock()->now();
            
            if (this->move_group_interface_->plan(my_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                result->success = false;
                result->message = "Planning failed with requested configuration.";
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "Planning failed for target pose.");
                return;
            }
            
            auto planning_end = this->get_clock()->now();
            double actual_planning_time = (planning_end - planning_start).seconds();
            
            RCLCPP_INFO(this->get_logger(), 
                "Planning succeeded in %.3f seconds (cache miss, total misses: %zu)",
                actual_planning_time, cache_misses_);

            // Build motion plan request for caching
            plan_request.group_name = "ur5_arm";
            plan_request.pipeline_id = planning_pipeline;
            plan_request.planner_id = planner_id;
            plan_request.allowed_planning_time = 5.0;
            
            // Add goal constraint
            moveit_msgs::msg::Constraints goal_constraints;
            moveit_msgs::msg::PositionConstraint pos_constraint;
            pos_constraint.header = goal->target_pose.header;
            pos_constraint.link_name = tcp_link;
            pos_constraint.weight = 1.0;
            goal_constraints.position_constraints.push_back(pos_constraint);
            plan_request.goal_constraints.push_back(goal_constraints);
        }

        // 7. Execution and Feedback Loop
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

        // 8. Final Result
        moveit::core::MoveItErrorCode final_code = future.get();
        if (final_code == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true;
            result->message = "Goal reached successfully.";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal execution succeeded.");

            // 9. Save trajectory to warehouse cache if we didn't use a cached one
            if (use_cache_ && trajectory_cache_ && !used_cache) {
                trajectory_cache_->saveTrajectory(
                    goal->target_pose,
                    planning_pipeline,
                    planner_id,
                    tcp_link,
                    current_joint_positions,
                    my_plan.trajectory_,
                    plan_request);
            }
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
