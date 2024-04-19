# Franka_Panda_Robot_Moveit2
A Franka Emika Panda Robot is used to perform a Pick and Place task in RViz using Moveit2.

The ROS distribution being used is ROS Humble.


## GitHub Link
https://github.com/lorocks/Franka_Panda_Robot_Moveit2


## Installing Dependencies
The following libraires will be required to use the ROS package.

### Dependencies - Panda_Robot
1. tf2_ros
2. moveit_ros_move_group
3. ros2_controller

### Dependencies - Panda_Planner
1. moveit_core (Humble)
2. moveit_msgs
3. moveit_ros_planning
4. moveit_ros_planning_interface
5. moveit_task_constructor_core
6. moveit_task_constructor_msgs


## Building the Packages
Note:- If the above mentioned dependencies are not installed the build will fail.

To successfully build this Project, moveit for Humble is required.

First clone the repository and then follow the commands listed below,

```bash
# Before execution ensure ROS Humble underlay has been sourced

# Source the Moveit Humble packages
  source <moveit Directory>/install/setup.sh

# Move to the Project directory and build
  colcon build
```


## Packages
There are two packages, 
    - panda_robot: used to spawn the robot in RVix
    - panda_planner: used to plan for a pick and place task

### Panda_Robot
This package consists of urdf and meshes for the Panda robot, and configuration files to spawn and use the robot.

#### Launch Robot
The robot can be launched using the following command,
```bash
# Naviagte into the Project directory
# Source the Project
  source install/setup.sh
# Launch Panda robot in RViz
  ros2 launch panda_robot my_demo.launch.py
```

This should spawn a robot in RViz with Motion Task Planning tab enabled.

To input a custom path for planning and execution, add the tab manually from within RViz.

### Panda_Planner
This package consists of planning methods and task to be executed to succesfully complete a pick and place operation.

#### Perform Pick and Place
##### Pick and Place
```bash
# Naviagte into the Project directory
# Source the Project
  source install/setup.sh
# Launch the planner after launching the robot in RViz
  ros2 launch panda_planner brute_task.launch.py
```

Wait patiently for the task to be executed.

##### Random Actions
```bash
# Naviagte into the Project directory
# Source the Project
  source install/setup.sh
# Launch the planner after launching the robot in RViz
  ros2 launch panda_planner single.launch.py
```

#### Possible Errors
There is a low chance that the planning might fail midway because RViz Scene changes and the introduction of some unregistered collision.

If this occurs, close the planner and RViz, and proceed to relaunch the panda_robot followed by launching the planner.


## Video Link