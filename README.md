# Installation guidelines 
Install as a package under src repository 

```
git init 
git clone https://github.com/Greblovs/RobotArm.git master
```
# Usage without Arm
Firstly launch roscore 
```
roscore
```

Setup your Rviz visualization
```
roslaunch j2s7s300_moveit_config j2s7s300_virtual_robot_demo.launch
```
Compile C++ source files into executables
```
Catkin_make 
```
Finally, execute the program to plan path 
```
rosrun kinova_scripts runArc 
```
Do not forget to source to correct setup.bash before working with ros nodes 
```
source devel/setup.bash
```

# Usage with Arm 

start docker container
```
docker start <name>
```

console into docker workspace 
```
docker exec -it <name> bash
```

Install package from github 
```
cd <workspace_name>
cd src 
mkdir kinova_scripts 
cd kinova_scripts
git init 
git clone https://github.com/Greblovs/RobotArm.git master
```

After installing, do not forget to 
```
source devel/setup.bash
catkin_make
```

# Run code on Arm

```
roslaunch <name> (kinova_bringup) kinova_robot.launch kinova_robotType:=j2s7s300
roslaunch j2s7s300_moveit_config j2s7s300_demo.launch 
rosrun topic_tools replay /j2s7s300_driver/out/joint_state joint_states
rosrun kinova_scripts <script_name>
```


# Functionality and Tutorial 

This library is based on MoveIt package (https://ros-planning.github.io/moveit_tutorials/)

## Setup

First, include library and create a class instance  

```
#include "Planner.h"

ArmControl control("arm", "arm_link0"); 
``` 

## Basic Path Planning 

You can execute basic path planning by using plan_in_xyzw method  
plan_in_xyzw
  - [float] target x 
  - [float] target y 
  - [float] target z
  - [tf2:Quaternion] target rotation as quaternion
  - [geometry_msgs::Pose] current pose 
  - [bool] execute on robot (1- yes, 0 - no)
  - [int] max number of points in trajectory, to avoid invalid plans (20 is a good starting point) 

return 
  - [geometry_msgs::Pose] pose after function execution (!important, it may slightly vary from target pose)

  

```
geometry_msgs::Pose start = control.getCurrentPose();
tf2::Quaternion quat;
quat[1] = start.orientation.x;
quat[2] = start.orientation.y;
quat[3] = start.orientation.z;
quat[0] = start.orientation.w;
geometry_msgs::Pose start1 = control.plan_in_xyzw(0.5, 0.02, 0.05, quat, start, 1, 20);
```
## Gippers 

To close or open gripper use openGripper/closeGripper methods 
openGripper 
  - [geometry_msgs::Pose] current pose 

closeGripper 
  - [geometry_msgs::Pose] current pose 

```
control.closeGripper(start1);
```

## Collision Objects & planning constraints 

While executing path planning, it is extremly important to add planning constraints  
Use addColObject function to generate bounding boxes and other objects  
addColObject
  - [string] name (should be unique)
  - position x 
  - position y
  - position z
  - rotation (1 for 'vertical) 
  - width 
  - length 
  - height 

Here is a set of objects to execute path planning on the testbed  

```
control.addColObject("floor", 0.25, 0, -0.04, 1, 1.3, 1.2, 0.01);
control.addColObject("FrontWall", 0.9, 0, 0.5, 1, 0.01, 1.2, 1);
control.addColObject("BackWall", -0.4, 0, 0.5, 1, 0.01, 1.2, 1);
control.addColObject("LeftWall", 0.25, 0.6, 0.5, 1, 1.3, 0.01, 1);
control.addColObject("RightWall", 0.25, -0.6, 0.5, 1, 1.3, 0.01, 1);
control.addColObject("Camera", 0.65, 0.0, 0.65 + 0.175, 1, 0.1, 0.1, 0.35);
control.addColObject("TopWall", 0.25, 0, 1.0, 1, 1.28, 1.18, 0.01);
```



