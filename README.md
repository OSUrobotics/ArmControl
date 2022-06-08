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

## Basic Usage 

First, include library and create a class instance  

```
#include "Planner.h"

ArmControl control("arm", "arm_link0"); 
``` 

You can execute basic path planning by using plan_in_xyzw method  
Function takes several arguments: 
  - [float] target x 
  - [float] target y 
  - [float] target z
  - [tf2:Quaternion] target rotation as quaternion
  - [geometry_msgs::Pose] current pose 
  - [bool] execute on robot (1- yes, 0 - no)
  - [int] max number of points in trajectory, to avoid invalid plans (20 is a good starting point) 
  

```
geometry_msgs::Pose start = control.getCurrentPose();
tf2::Quaternion quat;
quat[1] = start.orientation.x;
quat[2] = start.orientation.y;
quat[3] = start.orientation.z;
quat[0] = start.orientation.w;
geometry_msgs::Pose start1 = control.plan_in_xyzw(0.5, 0.02, 0.05, quat, start, 1, 20);
```
