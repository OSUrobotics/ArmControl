# include "Planner.h"
/*
    Sergiy Greblov in 2022 
    Oregon State University 
*/

// based on Moveit documentation: 
// http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html

namespace rvt = rviz_visual_tools;


// initialize Rviz visualization 
bool ArmControll:: init_RViz(std::string link_name){
    this->visual_tools = new moveit_visual_tools::MoveItVisualTools(link_name);
    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl();
    return 0; 
}


// init all moveit vars for arm controll 
ArmControll:: ArmControll(std::string robot_name, std::string link_name){ 
    this->PLANNING_GROUP = robot_name;
    this->PLANNING_GROUP_GRIPPER = "gripper";
    this->move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    this->move_group_gripper = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER);
    this->joint_model_group = this->move_group->getCurrentState()->getJointModelGroup(this->PLANNING_GROUP);
    this->joint_model_group_gripper = this->move_group_gripper->getCurrentState()->getJointModelGroup(this->PLANNING_GROUP_GRIPPER);
    this->planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    init_RViz(link_name);
}

// destructor 
ArmControll:: ~ArmControll(){
    delete planning_scene_interface;
    delete move_group;
    delete visual_tools;
}


// print message to Rviz 
void ArmControll:: printMessage(std::string text){
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.5;
    this->visual_tools->publishText(text_pose, text, rvt::WHITE, rvt::XLARGE);
}


bool ArmControll::comparePoses(geometry_msgs::Pose first, geometry_msgs::Pose second, int precision){
    if(abs(first.position.x - second.position.x) > precision){
        return false;
    }
    if(abs(first.position.y - second.position.y) > precision){
        return false;
    }
    if(abs(first.position.z - second.position.z) > precision){
        return false;
    }
    return true;
}

void ArmControll:: verifyExecution(geometry_msgs::Pose target, int precision, bool execute){
    tf2::Quaternion quat;
    quat[0] = target.orientation.x;
    quat[1] = target.orientation.y;
    quat[2] = target.orientation.z;
    quat[3] = target.orientation.w;

    
    geometry_msgs::Pose current = this->getCurrentPose(); 
    if (this->comparePoses(current, target, precision) == false){
        std::cout << "executing correction" << std::endl;
        this->plan_in_xyzw(target.position.x, target.position.y, target.position.z, quat, current, execute);
    }
    else{
        std::cout << "No correction needed" << std::endl;
    }
}

// plan movement based on rotation and transition 
geometry_msgs::Pose ArmControll:: plan_in_xyzw(float x, float y, float z, tf2::Quaternion quat, geometry_msgs::Pose start_pose, bool execute,  int treshhold){
    moveit::core::RobotState start_state(*(this->move_group->getCurrentState()));
    start_state.setFromIK(joint_model_group, start_pose);
    move_group->setStartState(start_state);
    
    
    geometry_msgs::Pose target; 
    
    target.orientation.w = quat[0]; 
    target.orientation.x = quat[1]; 
    target.orientation.y = quat[2]; 
    target.orientation.z = quat[3];

    target.position.x = x; 
    target.position.y = y; 
    target.position.z = z;    
        
    this->move_group->setPoseTarget(target);
    this->move_group->setGoalTolerance(0.01);
    moveit::planning_interface::MoveGroupInterface::Plan target_plan;

    while (true){
        this->move_group->plan(target_plan);
        if (this->validatePlan(target_plan.trajectory_, treshhold)){
            break;
        }
        
    } 
    
    this->visual_tools->publishTrajectoryLine(target_plan.trajectory_, this->joint_model_group);
    this->visual_tools->trigger();
    visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    if (execute == 1){
        this->move_group->execute(target_plan);
    }
    this->verifyExecution(target, 0.02, execute);
    return this->getCurrentPose();
}

// plan movement using cartesian path comuting.
// points: points along which to construct path 
// execute: execute program on robot? 1 - yes, 0 no 
// showAny: show unsuccesfull trajectories in moveit? 1 - yes, 0 - no
float ArmControll:: plan_cartesian_path(std::vector<geometry_msgs::Pose> points, bool execute, bool showAny){
    this->move_group->setMaxVelocityScalingFactor(1);
    moveit_msgs::RobotTrajectory tr; 
    double jump_treshold = 2;
    double step = 0.01; 
    float result = this->move_group->computeCartesianPath(points, step, jump_treshold, tr);
    
    if (result == 1 || showAny){
        this->visual_tools->publishPath(points, rvt::LIME_GREEN, rvt::SMALL);
        this->saveTrajectory(tr, "PathOut.txt");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = tr;
        

     
        
    }
    
    this->visual_tools->trigger();
    return result;
}




bool ArmControll:: validatePlan(moveit_msgs::RobotTrajectory tr, int treshhold){
    if (tr.joint_trajectory.points.size() > treshhold){
        std::cout << "rejected with " << tr.joint_trajectory.points.size() << " points" << std::endl;
        return false; 
    }

    return true;

}


// save trajectory to file 
void ArmControll:: saveTrajectory(moveit_msgs::RobotTrajectory tr, char file_name[20]){
    std::ofstream f;
    f.open(file_name, std::ofstream::trunc);
    
    f << "m:Path Info\n";
    f << tr.joint_trajectory.joint_names.size();
    f << "\n";
    for (int i = 0; i < tr.joint_trajectory.joint_names.size(); i++){
        f << tr.joint_trajectory.joint_names[i];
        f << "\n";
    }
    f << tr.joint_trajectory.points.size();
    f << "\n";
    for (int i = 0; i < tr.joint_trajectory.points.size(); i++){
        f << tr.joint_trajectory.points[i].positions.size();
        f << "\n";
        for (int j = 0; j< tr.joint_trajectory.points[i].positions.size(); j++){
            f << tr.joint_trajectory.points[i].positions[j];
            f << " ";
        }
        f << "\n";
        f << tr.joint_trajectory.points[i].velocities.size();
        f << "\n";
        for (int j = 0; j< tr.joint_trajectory.points[i].velocities.size(); j++){
            f << tr.joint_trajectory.points[i].velocities[j];
            f << " ";
        }
        f << "\n";
        f << tr.joint_trajectory.points[i].accelerations.size();
        f << "\n";
        for (int j = 0; j< tr.joint_trajectory.points[i].accelerations.size(); j++){
            f << tr.joint_trajectory.points[i].accelerations[j];
            f << " ";
        }
        f << "\n";
        /*
        f << "JointTrajectory:Effort\n";
        for (int j = 0; j< tr.joint_trajectory.points[i].effort.size(); j++){
            f << tr.joint_trajectory.points[i].effort[j];
            f << " ";
        }
        */
        f << "m:JointTrajectory:TimeFromStart\n";
        f << tr.joint_trajectory.points[i].time_from_start; 
        f << "\n";
    }
    f.close();
}

// load trajectory from file
moveit_msgs::RobotTrajectory ArmControll:: readTrajectory(){
    moveit_msgs::RobotTrajectory tr; 
    double holder; 
    std::ifstream f("PathOut.txt");
    std::string line;
    getline (f,line);
    int size; 
    f >> size; 
    getline(f, line);
    for (int i = 0; i < size; i++){
        getline(f, line);
        tr.joint_trajectory.joint_names.push_back(line);
    }
    int numPoints;
    f >> numPoints;
    tr.joint_trajectory.points.resize(numPoints);
    for (int i = 0; i < numPoints; i++){
        f >> size;
        for (int j= 0; j < size; j++){
            f >> holder;
            tr.joint_trajectory.points[i].positions.push_back(holder);
        }
        f >> size;
        for (int j= 0; j < size; j++){
            f >> holder;
            tr.joint_trajectory.points[i].velocities.push_back(holder);
        }
        f >> size;
        for (int j= 0; j < size; j++){
            f >> holder;
            tr.joint_trajectory.points[i].accelerations.push_back(holder);
        }
        getline(f, line);
        getline(f, line);
        f >> holder;
        tr.joint_trajectory.points[i].time_from_start = (ros::Duration) holder; 
    }
    f.close();
    this->saveTrajectory(tr, "PathOut1.txt");
    return tr; 
}


// print current coordinates of the arm 
void ArmControll:: print_current_pose_position(){
    geometry_msgs::Pose current; 
    current = this->move_group->getCurrentPose().pose;
    ROS_INFO("----------------------------------\n");
    ROS_INFO_NAMED("Current_Pose_Position", "x: %f", current.position.x);
    ROS_INFO_NAMED("Current_Pose_Position", "y: %f", current.position.y);
    ROS_INFO_NAMED("Current_Pose_Position\n", "z: %f", current.position.z);
    ROS_INFO("----------------------------------\n");
}


// print current rotation of the arm (in quaternion xyzw)
void ArmControll:: print_current_pose_orientation(){
    geometry_msgs:: Pose current; 
    current = this->move_group->getCurrentPose().pose;
    ROS_INFO("----------------------------------\n");
    ROS_INFO_NAMED("Current_Pose_Orientation", "x: %f", current.orientation.x);
    ROS_INFO_NAMED("Current_Pose_Orientation", "y: %f", current.orientation.y);
    ROS_INFO_NAMED("Current_Pose_Orientation", "z: %f", current.orientation.z);
    ROS_INFO_NAMED("Current_Pose_Orientation\n", "w: %f", current.orientation.w);
    ROS_INFO("----------------------------------\n");
}

// return current pose 
geometry_msgs::Pose ArmControll:: getCurrentPose(){
    return this->move_group->getCurrentPose().pose; 
}


// return current pose 
geometry_msgs::Pose ArmControll:: getCurrentPoseGripper(){
    return this->move_group_gripper->getCurrentPose().pose; 
}

void ArmControll:: publishSphere(ros::NodeHandle &node_handle){
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker_array", 0 );
    visualization_msgs::Marker marker;
    marker.header.frame_id = this->move_group->getPlanningFrame(); 
    marker.id = 23;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
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


void ArmControll:: addColObject(std::string name, float x, float y, float z, float r, float l, float w, float h){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = this->move_group->getPlanningFrame();  
    collision_object.id = name;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = l;
    primitive.dimensions[1] = w;
    primitive.dimensions[2] = h;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = r;
    box_pose.position.x = x;
    box_pose.position.y = y;
    box_pose.position.z = z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    this->collision_list.push_back(name);
    this->planning_scene_interface->applyCollisionObjects(collision_objects);
}


void ArmControll:: deleteColObject(std::string name){
    std::vector<std::string> temp;
    temp.push_back(name); 
    this->planning_scene_interface->removeCollisionObjects(temp);
}

void ArmControll:: deleteAllObjects(){
    this->planning_scene_interface->removeCollisionObjects(this->collision_list);   
}


void ArmControll:: closeGripper(geometry_msgs::Pose start_pose){
    moveit::planning_interface::MoveGroupInterface::Plan target_gripper_plan; 
    moveit::core::RobotState start_state(*(this->move_group->getCurrentState()));
    start_state.setFromIK(this->joint_model_group, start_pose);
    move_group_gripper->setStartState(start_state);


    this->move_group_gripper->setGoalTolerance(0.01);
    this->move_group_gripper->setJointValueTarget(this->move_group_gripper->getNamedTargetValues("Close"));
    this->move_group_gripper->plan(target_gripper_plan);

    this->visual_tools->publishTrajectoryLine(target_gripper_plan.trajectory_, this->joint_model_group_gripper);
    this->visual_tools->trigger();
    visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}

void ArmControll:: openGripper(geometry_msgs::Pose start_pose){
    moveit::core::RobotState start_state(*(this->move_group->getCurrentState()));
    start_state.setFromIK(this->joint_model_group, start_pose);
    move_group_gripper->setStartState(start_state);
    moveit::planning_interface::MoveGroupInterface::Plan target_gripper_plan; 
    this->move_group_gripper->setGoalTolerance(0.01);
    this->move_group_gripper->setJointValueTarget(this->move_group_gripper->getNamedTargetValues("Open"));
    this->move_group_gripper->plan(target_gripper_plan);

    this->visual_tools->publishTrajectoryLine(target_gripper_plan.trajectory_, this->joint_model_group_gripper);
    this->visual_tools->trigger();
    visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}
