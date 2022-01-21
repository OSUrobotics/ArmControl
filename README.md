# Installation guidelines 
Install as a package under src repository 

```
git init 
git clone https://github.com/Greblovs/RobotArm.git master
```
# Usage 
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
Do not forget to setup source to correct setup.bash before working with ros nodes 
```
source devel/setup.bash
```
