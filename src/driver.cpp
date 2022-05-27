# include "Planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Arc");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // setup arm joints and the scene
    
    ArmControll control("arm", "arm_link0");
  
    control.addColObject("floor", 0.25, 0, 0.05, 1, 1.3, 1.2, 0.01);
    control.addColObject("FrontWall", 0.9, 0, 0.5, 1, 0.01, 1.2, 1);
    control.addColObject("BackWall", -0.4, 0, 0.5, 1, 0.01, 1.2, 1);
    control.addColObject("LeftWall", 0.25, 0.6, 0.5, 1, 1.3, 0.01, 1);
    control.addColObject("RightWall", 0.25, -0.6, 0.5, 1, 1.3, 0.01, 1);
    control.addColObject("Camera", 0.65, 0.0, 0.65+0.175, 1, 0.1, 0.1, 0.35);
    control.addColObject("TopWall", 0.25, 0, 1.0, 1, 1.28, 1.18, 0.01);



   

    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0*PI/180);    
    geometry_msgs::Pose start = control.getCurrentPose();
  
    quat[0] = start.orientation.x;
    quat[1] = start.orientation.y;
    quat[2] = start.orientation.z;
    quat[3] = start.orientation.w;

    //quat[0] = 0.68463;
    //quat[1] = -0.22436;
    //quat[2] = 0.68808;
    //quat[3] = 0.086576;

    geometry_msgs::Pose start1 = control.plan_in_xyzw(0.5, 0, 0.2, quat, start, 0);

    
   

    geometry_msgs::Pose start2 = control.plan_in_xyzw(start1.position.x+0.2, start1.position.y, start1.position.z, quat, start1, 0 15);
    control.closeGripper(start2);

   
   
    control.deleteAllObjects();
}
