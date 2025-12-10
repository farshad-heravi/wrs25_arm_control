/**
 * @file go_to_pose_action_server.cpp
 * @brief GoToPose action server with trajectory caching support
 * 
 * This action server plans and executes robot arm movements to target poses.
 * It includes automatic trajectory caching using a file-based storage system
 * to reuse previously computed plans when the start state and goal are similar.
 * 
 * TRAJECTORY CACHING:
 * - Trajectories are stored as binary files in ~/.ros/trajectory_cache/
 * - Cached trajectories are automatically retrieved when a matching query is found
 * - Successfully executed trajectories are automatically saved
 * - Cache files can be cleared by deleting the cache directory
 * 
 * PARAMETERS:
 * - use_trajectory_cache (bool, default: true): Enable/disable trajectory caching
 * - cache_tolerance_position (double, default: 0.003): Position tolerance for cache matching (m)
 * - cache_tolerance_orientation (double, default: 0.01): Orientation tolerance for cache matching (rad)
 * - cache_tolerance_joint (double, default: 0.05): Joint state tolerance for cache matching (rad)
 * - trajectory_cache_dir (string): Path to cache directory
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
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "wrs25_arm_actions/action/go_to_pose.hpp"

namespace wrs25_arm_actions
{

/**
 * @brief Simple file-based trajectory cache
 * 
 * Stores trajectories as binary files, avoiding the warehouse_ros_sqlite schema bugs.
 */
class FileTrajectoryCache
{
public:
    explicit FileTrajectoryCache(
        const rclcpp::Logger& logger,
        const std::string& cache_dir = "")
        : logger_(logger)
    {
        // Set default cache directory
        if (cache_dir.empty()) {
            const char* home = std::getenv("HOME");
            if (home) {
                cache_dir_ = std::string(home) + "/.ros/trajectory_cache";
            } else {
                cache_dir_ = "/tmp/trajectory_cache";
            }
        } else {
            cache_dir_ = cache_dir;
        }

        // Set default tolerances
        position_tolerance_ = 0.003;      // 3mm
        orientation_tolerance_ = 0.01;    // ~0.5 degrees
        joint_tolerance_ = 0.05;          // ~3 degrees

        // Create cache directory if it doesn't exist
        try {
            std::filesystem::create_directories(cache_dir_);
            RCLCPP_INFO(logger_, "✓ Trajectory cache initialized at: %s", cache_dir_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(logger_, "Failed to create cache directory: %s", e.what());
        }
    }

    void setTolerances(double position_tol, double orientation_tol, double joint_tol)
    {
        position_tolerance_ = position_tol;
        orientation_tolerance_ = orientation_tol;
        joint_tolerance_ = joint_tol;
    }

    /**
     * @brief Generate a unique cache key based on goal parameters
     */
    std::string generateCacheKey(
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const std::string& planning_pipeline,
        const std::string& planner_id,
        const std::string& tcp_link) const
    {
        std::stringstream ss;
        
        // Use discrete bins for position (based on tolerance)
        int bin_size_mm = static_cast<int>(position_tolerance_ * 1000);
        if (bin_size_mm < 1) bin_size_mm = 1;
        
        int px = static_cast<int>(goal_pose.pose.position.x * 1000) / bin_size_mm;
        int py = static_cast<int>(goal_pose.pose.position.y * 1000) / bin_size_mm;
        int pz = static_cast<int>(goal_pose.pose.position.z * 1000) / bin_size_mm;
        
        // Use discrete bins for orientation (based on tolerance)
        int bin_size_ori = static_cast<int>(orientation_tolerance_ * 100);
        if (bin_size_ori < 1) bin_size_ori = 1;
        
        int qw = static_cast<int>(goal_pose.pose.orientation.w * 100) / bin_size_ori;
        int qx = static_cast<int>(goal_pose.pose.orientation.x * 100) / bin_size_ori;
        int qy = static_cast<int>(goal_pose.pose.orientation.y * 100) / bin_size_ori;
        int qz = static_cast<int>(goal_pose.pose.orientation.z * 100) / bin_size_ori;
        
        // Sanitize strings for filename
        auto sanitize = [](const std::string& s) {
            std::string result;
            for (char c : s) {
                if (std::isalnum(c) || c == '_' || c == '-') {
                    result += c;
                } else {
                    result += '_';
                }
            }
            return result;
        };
        
        ss << sanitize(goal_pose.header.frame_id) << "_"
           << px << "_" << py << "_" << pz << "_"
           << qw << "_" << qx << "_" << qy << "_" << qz << "_"
           << sanitize(planning_pipeline) << "_" 
           << sanitize(planner_id) << "_" 
           << sanitize(tcp_link);
        
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
     * @brief Try to fetch a cached trajectory
     */
    bool fetchTrajectory(
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const std::string& planning_pipeline,
        const std::string& planner_id,
        const std::string& tcp_link,
        const std::vector<double>& current_joint_positions,
        moveit_msgs::msg::RobotTrajectory& trajectory_out)
    {
        std::string cache_key = generateCacheKey(goal_pose, planning_pipeline, planner_id, tcp_link);
        std::string cache_file = cache_dir_ + "/" + cache_key + ".traj";

        if (!std::filesystem::exists(cache_file)) {
            RCLCPP_DEBUG(logger_, "No cached trajectory found for key: %s", cache_key.c_str());
            return false;
        }

        try {
            std::ifstream ifs(cache_file, std::ios::binary);
            if (!ifs.is_open()) {
                RCLCPP_DEBUG(logger_, "Failed to open cache file: %s", cache_file.c_str());
                return false;
            }

            // Read number of joints
            size_t num_joints;
            ifs.read(reinterpret_cast<char*>(&num_joints), sizeof(num_joints));

            // Read joint names
            trajectory_out.joint_trajectory.joint_names.resize(num_joints);
            for (size_t i = 0; i < num_joints; ++i) {
                size_t name_len;
                ifs.read(reinterpret_cast<char*>(&name_len), sizeof(name_len));
                trajectory_out.joint_trajectory.joint_names[i].resize(name_len);
                ifs.read(&trajectory_out.joint_trajectory.joint_names[i][0], name_len);
            }

            // Read number of points
            size_t num_points;
            ifs.read(reinterpret_cast<char*>(&num_points), sizeof(num_points));

            trajectory_out.joint_trajectory.points.resize(num_points);
            for (size_t i = 0; i < num_points; ++i) {
                auto& point = trajectory_out.joint_trajectory.points[i];
                
                // Read positions
                point.positions.resize(num_joints);
                ifs.read(reinterpret_cast<char*>(point.positions.data()), num_joints * sizeof(double));
                
                // Read velocities
                point.velocities.resize(num_joints);
                ifs.read(reinterpret_cast<char*>(point.velocities.data()), num_joints * sizeof(double));
                
                // Read accelerations
                point.accelerations.resize(num_joints);
                ifs.read(reinterpret_cast<char*>(point.accelerations.data()), num_joints * sizeof(double));
                
                // Read time_from_start
                int32_t sec, nanosec;
                ifs.read(reinterpret_cast<char*>(&sec), sizeof(sec));
                ifs.read(reinterpret_cast<char*>(&nanosec), sizeof(nanosec));
                point.time_from_start.sec = sec;
                point.time_from_start.nanosec = nanosec;
            }

            ifs.close();

            // Validate start state matches current state
            if (!trajectory_out.joint_trajectory.points.empty()) {
                const auto& start_positions = trajectory_out.joint_trajectory.points[0].positions;
                std::vector<double> start_vec(start_positions.begin(), start_positions.end());
                
                if (!jointStatesMatch(start_vec, current_joint_positions)) {
                    RCLCPP_DEBUG(logger_, "Cached trajectory start state doesn't match current state");
                    return false;
                }
            }

            RCLCPP_INFO(logger_, 
                "✓ CACHE HIT: Retrieved trajectory with %zu waypoints (key: %s)",
                trajectory_out.joint_trajectory.points.size(), cache_key.c_str());
            
            return true;

        } catch (const std::exception& e) {
            RCLCPP_WARN(logger_, "Exception reading cached trajectory: %s", e.what());
            return false;
        }
    }

    /**
     * @brief Save a trajectory to cache
     */
    bool saveTrajectory(
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const std::string& planning_pipeline,
        const std::string& planner_id,
        const std::string& tcp_link,
        const moveit_msgs::msg::RobotTrajectory& trajectory)
    {
        std::string cache_key = generateCacheKey(goal_pose, planning_pipeline, planner_id, tcp_link);
        std::string cache_file = cache_dir_ + "/" + cache_key + ".traj";

        // Skip if already exists
        if (std::filesystem::exists(cache_file)) {
            RCLCPP_DEBUG(logger_, "Trajectory already cached for key: %s", cache_key.c_str());
            return true;
        }

        try {
            std::ofstream ofs(cache_file, std::ios::binary);
            if (!ofs.is_open()) {
                RCLCPP_WARN(logger_, "Failed to create cache file: %s", cache_file.c_str());
                return false;
            }

            const auto& jt = trajectory.joint_trajectory;
            
            // Write number of joints
            size_t num_joints = jt.joint_names.size();
            ofs.write(reinterpret_cast<const char*>(&num_joints), sizeof(num_joints));

            // Write joint names
            for (const auto& name : jt.joint_names) {
                size_t name_len = name.size();
                ofs.write(reinterpret_cast<const char*>(&name_len), sizeof(name_len));
                ofs.write(name.data(), name_len);
            }

            // Write number of points
            size_t num_points = jt.points.size();
            ofs.write(reinterpret_cast<const char*>(&num_points), sizeof(num_points));

            // Write each point
            for (const auto& point : jt.points) {
                // Write positions (pad if necessary)
                std::vector<double> positions = point.positions;
                positions.resize(num_joints, 0.0);
                ofs.write(reinterpret_cast<const char*>(positions.data()), num_joints * sizeof(double));
                
                // Write velocities (pad if necessary)
                std::vector<double> velocities = point.velocities;
                velocities.resize(num_joints, 0.0);
                ofs.write(reinterpret_cast<const char*>(velocities.data()), num_joints * sizeof(double));
                
                // Write accelerations (pad if necessary)
                std::vector<double> accelerations = point.accelerations;
                accelerations.resize(num_joints, 0.0);
                ofs.write(reinterpret_cast<const char*>(accelerations.data()), num_joints * sizeof(double));
                
                // Write time_from_start
                int32_t sec = point.time_from_start.sec;
                int32_t nanosec = point.time_from_start.nanosec;
                ofs.write(reinterpret_cast<const char*>(&sec), sizeof(sec));
                ofs.write(reinterpret_cast<const char*>(&nanosec), sizeof(nanosec));
            }

            ofs.close();

            RCLCPP_INFO(logger_, 
                "✓ CACHE SAVE: Saved trajectory with %zu waypoints (key: %s)",
                jt.points.size(), cache_key.c_str());
            
            return true;

        } catch (const std::exception& e) {
            RCLCPP_WARN(logger_, "Exception saving trajectory to cache: %s", e.what());
            return false;
        }
    }

    size_t getCacheSize() const
    {
        try {
            size_t count = 0;
            for (const auto& entry : std::filesystem::directory_iterator(cache_dir_)) {
                if (entry.path().extension() == ".traj") {
                    ++count;
                }
            }
            return count;
        } catch (...) {
            return 0;
        }
    }

    void clearCache()
    {
        try {
            for (const auto& entry : std::filesystem::directory_iterator(cache_dir_)) {
                if (entry.path().extension() == ".traj") {
                    std::filesystem::remove(entry.path());
                }
            }
            RCLCPP_INFO(logger_, "Trajectory cache cleared");
        } catch (const std::exception& e) {
            RCLCPP_WARN(logger_, "Failed to clear cache: %s", e.what());
        }
    }

private:
    rclcpp::Logger logger_;
    std::string cache_dir_;
    
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
        this->declare_parameter("trajectory_cache_dir", "");
        
        // Get parameters
        use_cache_ = this->get_parameter("use_trajectory_cache").as_bool();
        double pos_tol = this->get_parameter("cache_tolerance_position").as_double();
        double ori_tol = this->get_parameter("cache_tolerance_orientation").as_double();
        double joint_tol = this->get_parameter("cache_tolerance_joint").as_double();
        std::string cache_dir = this->get_parameter("trajectory_cache_dir").as_string();

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

        // Initialize trajectory cache
        if (use_cache_) {
            trajectory_cache_ = std::make_unique<FileTrajectoryCache>(this->get_logger(), cache_dir);
            trajectory_cache_->setTolerances(pos_tol, ori_tol, joint_tol);
        }
    }

private:
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<FileTrajectoryCache> trajectory_cache_;
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

        // 5. Try to fetch from cache first
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

            // 9. Save trajectory to cache if we didn't use a cached one
            if (use_cache_ && trajectory_cache_ && !used_cache) {
                trajectory_cache_->saveTrajectory(
                    goal->target_pose,
                    planning_pipeline,
                    planner_id,
                    tcp_link,
                    my_plan.trajectory_);
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
