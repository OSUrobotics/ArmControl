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


