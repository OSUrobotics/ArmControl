#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h> 
#include <rosbag/bag.h>
#include <iostream>
#include <fstream>
#define PI 3.14159265



# include "Planner.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "State");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0*PI/180); 
    ArmControll control("arm", "arm_link0");
    geometry_msgs::Pose c = control.getCurrentPose();
    c.position.z = c.position.z + 0.2;
    c.position.z = c.position.x + 0.2;
    quat[0] = c.orientation.x;
    quat[1] = c.orientation.y;
    quat[2] = c.orientation.z;
    quat[3] = c.orientation.w;
    control.plan_in_xyzw(c.position.x, c.position.y, c.position.z, quat);
}