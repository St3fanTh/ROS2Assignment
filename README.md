````markdown
# my_robot_control_ur5

This repository provides a ROS 2 package for controlling a UR5 robot arm with MoveIt and ros2_control. It includes a demonstration launch file and a Cartesian interpolation node that computes, visualizes, and prints trajectories to given target poses.

## Prerequisites

- Ubuntu 24.04 LTS (tested in WSL)
- ROS 2 Jazzy
- Python 3.12
- MoveIt (running on `/compute_ik` and `/compute_fk` services)
- A ros2_control-compatible UR5 controller (`/ur5_arm_controller/follow_joint_trajectory`)

## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ws_moveit/src
   git clone https://github.com/St3fanTh/ROS2Assignment.git my_robot_control_ur5
````

2. Build the workspace:
   ```bash
   cd ~/ws_moveit
   colcon build 
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Getting Started

1. Launch the demo environment (robot, controllers, and MoveIt):
   ```bash
   ros2 launch my_robot_control_ur5 demo.launch.py
   ```
2. In a new terminal (after sourcing), run the Cartesian interpolation node with your target pose:
   ```bash
   ros2 run my_robot_control movelt_planning_obstacle 0.4 0.4 0.2 0.5 1.57 0
   ```
   - `<x> <y> <z>`: Cartesian position in meters
   - `<roll> <pitch> <yaw>`: Orientation in radians

This will:

- Compute inverse kinematics for each interpolated waypoint
- Publish visualization markers on `/waypoint_markers`
- Print each waypoint and joint solution to the console
- Send the joint trajectory to `/ur5_arm_controller/follow_joint_trajectory`

## Nodes and Scripts

- **demo.launch.py**: Launches the UR5 robot in simulation or on real hardware, brings up controllers, and MoveIt services.
- **movelt\_planning\_obstacle.py**: Node that reads a target pose, interpolates a Cartesian path, requests IK/FK, publishes markers, and executes the trajectory.

> **Note:** The following scripts were used for testing purposes and are **not** relevant to the main demo:
>
> - `add_box_obstacle.py`: publishes a green square obstacle
> - `cartesian_commander_panda.py` & `cartesian_commander_ur5.py`: Cartesian commanding test scripts for Panda and UR5 arms
> - `home_commander_panda.py` & `home_commander_ur5.py`: Home position commanding test scripts for Panda and UR5 arms

## Troubleshooting

- Ensure MoveIt IK/FK services (`/compute_ik`, `/compute_fk`) are available.
- Verify the controller `/ur5_arm_controller/follow_joint_trajectory` is active.
- If no `/joint_states` are received, the node will default to a pre-defined home pose.

## Tests

To run the unit tests for this package, use the following command:
   ```bash
python3 -m unittest discover -s test -v
   ```
Ensure you have all dependencies installed and sourced before running the tests.

