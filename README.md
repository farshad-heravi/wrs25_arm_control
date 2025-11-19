# WRS25 Arm Control

## Installation

There are two ways to install the required dependencies:

### 1. Using apt (Recommended)

This method is recommended for a stable setup.

```bash
sudo apt update
sudo apt install ros-humble-moveit ros-humble-ur
```

### 2. From source

This method allows you to have the latest versions of the packages.

1.  **Import repositories:**
    ```bash
    vcs import src < src/wrs25_arm_control/tools.repos
    ```

2.  **Install dependencies:**
    ```bash
    rosdep install --from-paths src -y --ignore-src
    ```

3.  **Build the workspace:**
    ```bash
    colcon build --symlink-install --cmake-args --executor sequential
    source install/setup.bash
    ```

## Launches

### View the system in RViz

```
ros2 launch wrs_cell_description start_rviz.launch.py
```
### MoveIt Launch
```
ros2 launch wrs_cell_description start_moveit.launch.py
```
### Launch Action Servers
```
ros2 launch wrs25_arm_actions go_to_pose_action.launch.py
```
### Launch MoveIt with Real Robot
```
ros2 launch wrs25_moveit_config start_moveit.launch.py use_real_robot:=true robot_ip:=192.168.1.101
```
add `fake_robotiq_gripper:=true` if you are not connected to real Robotiq gripper.

## Actions

This package provides two actions to control the arm and the gripper.

### 1. GoToPose

This action moves the arm to a desired pose.

**Action:** `/go_to_pose`

**Type:** `wrs25_arm_actions/action/GoToPose`

**Goal:**

*   `target_pose` (`geometry_msgs/PoseStamped`): The desired pose for the end-effector.
*   `planning_pipeline_id` (`string`): e.g., "ompl", "pilz_industrial_motion_planner", "chomp"
*   `planner_id` (`string`): e.g. "RRTkConfigDefault", "LIN", "PTP", "CHOMP"

**Example: [to be revised]**

```bash
ros2 action send_goal /go_to_pose wrs25_arm_actions/action/GoToPose '{ "target_pose": { "header": { "frame_id": "world" }, "pose": { "position": { "x": 0.3, "y": 0.3, "z": 0.3 }, "orientation": { "w": 1.0 } } } }'
```

### 2. GripperControl

This action controls the gripper.

**Action:** `/gripper_control`

**Type:** `wrs25_arm_actions/action/GripperControl`

**Goal:**

*   `position` (`float64`): The desired position for the gripper (0.0 for open, 0.8 for closed).

**Examples:**

*   **Open the gripper:**
    ```bash
    ros2 action send_goal /gripper_control wrs25_arm_actions/action/GripperControl '{ "position": 0.0 }'
    ```
*   **Close the gripper:**
    ```bash
    ros2 action send_goal /gripper_control wrs25_arm_actions/action/GripperControl '{ "position": 0.8 }'
    ```

### Using the Action Clients

Alternatively, you can use the provided action clients to send goals from the command line.

*   **`go_to_pose_action_client`**

    This client sends a goal to the `GoToPose` action server.

    **Example:**
    for ompl:
    ```bash
    ros2 run wrs25_arm_actions go_to_pose_action_client_node \
                --ros-args -p posx:=0.5 -p posy:=0.1 -p posz:=0.8 \
                -p orx:=1.0 -p ory:= 0.0 -p orz:=0.0 -p orw:=0.0 \
                -p pipeline:="ompl" \
                -p planner:="RRTConnectkConfigDefault" \
                -p tcp_link:="ur5_gripper_tcp"
    ```
    for pilz-LIN
    ```
    ros2 run wrs25_arm_actions go_to_pose_action_client_node \
                --ros-args -p posx:=0.5 -p posy:=0.1 -p posz:=0.8 \
               -p pipeline:="pilz_industrial_motion_planner" \
               -p orx:=1.0 -p ory:=0.0 -p orz:=0.0 -p orw:=0.0 \
               -p planner:="LIN" \
               -p tcp_link:="ur5_suction_tcp"
    ```

*   **`gripper_control_action_client`**

    This client sends a goal to the `GripperControl` action server.

    **Example (close gripper):**

    ```bash
    ros2 run wrs25_arm_actions gripper_control_action_client --ros-args -p position:=0.8
    ```
