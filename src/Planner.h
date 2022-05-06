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


# define PI 3.14159265


class ArmControll{
    private:
        std::string PLANNING_GROUP;
        moveit::planning_interface::PlanningSceneInterface scene;    
        const robot_state::JointModelGroup* joint_model_group; 
        moveit::planning_interface::MoveGroupInterface* move_group;
        moveit_visual_tools::MoveItVisualTools* visual_tools;
        bool init_RViz(std::string link_name);

      public:
        ArmControll(std::string robot_name, std::string link_name);
        ~ArmControll();
        void printMessage(std::string text);
        void plan_in_xyzw(float x, float y, float z, tf2::Quaternion quat);
        float plan_cartesian_path(std::vector<geometry_msgs::Pose> points, bool execute = 0, bool showAny = 0);
        void saveTrajectory(moveit_msgs::RobotTrajectory tr, char file_name[20]); 
        moveit_msgs::RobotTrajectory readTrajectory();
        void print_current_pose_position();
        void print_current_pose_orientation();
        geometry_msgs::Pose getCurrentPose();
        void publishSphere();
};