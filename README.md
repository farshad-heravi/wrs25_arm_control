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
    colcon build --symlink-install --cmake-args --executor secuential
    source install/setup.bash
    ```

## Usage

To illustrate the work cell

```
ros2 launch wrs_cell_description start_rviz.launch.py
```

## Actions

This package provides two actions to control the arm and the gripper.

### 1. GoToPose

This action moves the arm to a desired pose.

**Action:** `/go_to_pose`

**Type:** `wrs25_arm_actions/action/GoToPose`

**Goal:**

*   `target_pose` (`geometry_msgs/PoseStamped`): The desired pose for the end-effector.

**Example:**

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

### Launching the Action Servers

To use the actions, you need to launch the action servers:

```bash
ros2 launch wrs25_arm_actions go_to_pose_action.launch.py
```

### Using the Action Clients

Alternatively, you can use the provided action clients to send goals from the command line.

*   **`go_to_pose_action_client`**

    This client sends a goal to the `GoToPose` action server.

    **Example:**

    ```bash
    ros2 run wrs25_arm_actions go_to_pose_action_client --ros-args -p posx:=0.3 -p posy:=0.3 -p posz:=0.3
    ```

*   **`gripper_control_action_client`**

    This client sends a goal to the `GripperControl` action server.

    **Example (close gripper):**

    ```bash
    ros2 run wrs25_arm_actions gripper_control_action_client --ros-args -p position:=0.8
    ```
