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
