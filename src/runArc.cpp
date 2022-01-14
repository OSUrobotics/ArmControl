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

#define PI 3.14159265

/*
    Sergiy Greblov in 2022 
    Oregon State University 
*/


// based on Moveit documentation: 
// http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html

namespace rvt = rviz_visual_tools;

class ArmControll{
    private:

    std::string PLANNING_GROUP;
    moveit::planning_interface::PlanningSceneInterface scene;    
    const robot_state::JointModelGroup* joint_model_group; 
    moveit::planning_interface::MoveGroupInterface* move_group;
    moveit_visual_tools::MoveItVisualTools* visual_tools;
  


    bool init_RViz(std::string link_name){
        this->visual_tools = new moveit_visual_tools::MoveItVisualTools(link_name);
        visual_tools->deleteAllMarkers();
        visual_tools->loadRemoteControl();
        return 0; 
    }

    public:

    ArmControll(std::string robot_name, std::string link_name){ 
        this->PLANNING_GROUP = robot_name;
        this->move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
        this->joint_model_group = this->move_group->getCurrentState()->getJointModelGroup(this->PLANNING_GROUP);
        init_RViz(link_name);
    }
    ~ArmControll(){
        delete move_group;
        delete visual_tools;
    }

    void printMessage(std::string text){
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.5;
        this->visual_tools->publishText(text_pose, text, rvt::WHITE, rvt::XLARGE);
    }

    void plan_in_xyzw(float x, float y, float z, tf2::Quaternion quat){
        geometry_msgs::Pose target; 
        
        target.orientation.w = quat[0]; 
        target.orientation.x = quat[1]; 
        target.orientation.y = quat[2]; 
        target.orientation.z = quat[3];

        target.position.x = x; 
        target.position.y = y; 
        target.position.z = z;    
        
         
        this->move_group->setPoseTarget(target);
        this->move_group->setGoalTolerance(0.1);
        moveit::planning_interface::MoveGroupInterface::Plan target_plan; 
        this->move_group->plan(target_plan);
        this->visual_tools->publishTrajectoryLine(target_plan.trajectory_, this->joint_model_group);
        this->visual_tools->trigger();
        visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
        
    }


    float plan_cartesian_path(std::vector<geometry_msgs::Pose> points, bool showAny = 0){
        this->move_group->setMaxVelocityScalingFactor(1);
        moveit_msgs::RobotTrajectory tr; 
        double jump_treshold = 2    ;
        double step = 0.01; 
        float result = this->move_group->computeCartesianPath(points, step, jump_treshold, tr);
        
        if (result == 1 || showAny){
            this->visual_tools->publishPath(points, rvt::LIME_GREEN, rvt::SMALL);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            my_plan.trajectory_ = tr;
            this->move_group->execute(my_plan);
           
        }
        
        this->visual_tools->trigger();
       
        
        
        return result;
    }

 
    void print_current_pose_position(){
        geometry_msgs::Pose current; 
        current = this->move_group->getCurrentPose().pose;
        ROS_INFO("----------------------------------\n");
        ROS_INFO_NAMED("Current_Pose_Position", "x: %f", current.position.x);
        ROS_INFO_NAMED("Current_Pose_Position", "y: %f", current.position.y);
        ROS_INFO_NAMED("Current_Pose_Position\n", "z: %f", current.position.z);
        ROS_INFO("----------------------------------\n");
    }

    void print_current_pose_orientation(){
        geometry_msgs:: Pose current; 
        current = this->move_group->getCurrentPose().pose;
        ROS_INFO("----------------------------------\n");
        ROS_INFO_NAMED("Current_Pose_Orientation", "x: %f", current.orientation.x);
        ROS_INFO_NAMED("Current_Pose_Orientation", "y: %f", current.orientation.y);
        ROS_INFO_NAMED("Current_Pose_Orientation", "z: %f", current.orientation.z);
        ROS_INFO_NAMED("Current_Pose_Orientation\n", "w: %f", current.orientation.w);
        ROS_INFO("----------------------------------\n");
    }

    geometry_msgs::Pose getCurrentPose(){
        return this->move_group->getCurrentPose().pose; 
    }

    void publishSphere(){
        ros::NodeHandle node_handle;
        ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 1;
        marker.pose.position.y = 1;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2365;
        marker.scale.y = 0.2365;
        marker.scale.z = 0.2365;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
       
        vis_pub.publish(marker);
    }


};





int main(int argc, char** argv)
{
    ros::init(argc, argv, "Move_arc");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // setup arm joints and the scene
    
    ArmControll control("arm", "arm_link0");
    control.print_current_pose_position();
    control.print_current_pose_orientation();

    //control.publishSphere();
    

   
    
    geometry_msgs::Pose c = control.getCurrentPose();
    
    float deg = 90; 
    float r = 0.2365; 
    std::vector<geometry_msgs::Pose> points;
    points.resize(deg+1);
    points[0] = control.getCurrentPose();
    for (int i = 0; i<deg; i++){
        points[i+1] = points[0];
        points[i+1].position.y = points[0].position.y - r*sin(i*PI/180);
        points[i+1].position.x = points[0].position.x + (r - r*cos(i*PI/180));
        /*
        ROS_INFO_NAMED("Current_Pose_Orientation", "x: %f", points[i+1].position.x);
        ROS_INFO_NAMED("Current_Pose_Orientation", "y: %f", points[i+1].position.y);
        */
    }
    

    std::vector<geometry_msgs::Pose> pointsB;
    pointsB.resize(deg+1);
    pointsB[0] = points.back();
    tf2::Quaternion normal;
    normal.setRPY(0, 0, 0);
    for (int i = 0; i<deg; i++){
        tf2::Quaternion quat;

        quat.setRPY(0, 0, i*PI/180); 
        pointsB[i+1] = pointsB[0];
        pointsB[i+1].position.y = pointsB[0].position.y + (r - r*cos(i*PI/180));
        pointsB[i+1].position.x = pointsB[0].position.x - r*sin(i*PI/180);
        pointsB[i+1].orientation.x += (normal[0] - quat[0]);
        pointsB[i+1].orientation.y += (normal[1] - quat[1]);
        pointsB[i+1].orientation.z += (normal[2] - quat[2]);
        pointsB[i+1].orientation.w += (normal[3] - quat[3]);
        /*
        ROS_INFO_NAMED("Current_Pose_Orientation", "x: %f", pointsB[i+1].position.x);
        ROS_INFO_NAMED("Current_Pose_Orientation", "y: %f", pointsB[i+1].position.y);
        */
        points.push_back(pointsB[i+1]);
    }

    float result = 0; 
    int count = 0;
    while (result != 1 || count == 10){ 
        
        result = control.plan_cartesian_path(points);

        ROS_INFO_NAMED("Planning", "planned: %f", result);
        count ++; 

    }
    
    
}