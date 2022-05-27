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
#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>


# define PI 3.14159265


class ArmControll{
    private:
        std::vector<std::string> collision_list; 
        std::string PLANNING_GROUP;
        std::string PLANNING_GROUP_GRIPPER;
        moveit::planning_interface::PlanningSceneInterface scene;    
        const robot_state::JointModelGroup* joint_model_group; 
        const robot_state::JointModelGroup* joint_model_group_gripper; 
        moveit::planning_interface::MoveGroupInterface* move_group;
        moveit::planning_interface::MoveGroupInterface* move_group_gripper;
        moveit_visual_tools::MoveItVisualTools* visual_tools;
        moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;
        bool init_RViz(std::string link_name);

      public:
        ArmControll(std::string robot_name, std::string link_name);
        ~ArmControll();
        void printMessage(std::string text);
        geometry_msgs::Pose plan_in_xyzw(float x, float y, float z, tf2::Quaternion quat, geometry_msgs::Pose start_pose,  bool execute, int treshhold = 20);
        float plan_cartesian_path(std::vector<geometry_msgs::Pose> points, bool execute = 0, bool showAny = 0);
        void saveTrajectory(moveit_msgs::RobotTrajectory tr, char file_name[20]); 
        moveit_msgs::RobotTrajectory readTrajectory();
        void print_current_pose_position();
        void print_current_pose_orientation();
        geometry_msgs::Pose getCurrentPose();
        geometry_msgs::Pose getCurrentPoseGripper();
        void publishSphere(ros::NodeHandle &node_handle);
        void addColObject(std::string name, float x, float y, float z, float r, float l, float w, float h);

        void deleteColObject(std::string name);
        void deleteAllObjects();

        bool comparePoses(geometry_msgs::Pose first, geometry_msgs::Pose second, int precision);
        void closeGripper( geometry_msgs::Pose start_pose);
        void openGripper( geometry_msgs::Pose start_pose);
        void verifyExecution(geometry_msgs::Pose target, int precision, bool execute);
        bool validatePlan(moveit_msgs::RobotTrajectory tr, int treshhold);

};