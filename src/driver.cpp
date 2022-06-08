#include "Planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Arc");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // setup arm joints and the scene

    ArmControll control("arm", "arm_link0");

    control.addColObject("floor", 0.25, 0, -0.04, 1, 1.3, 1.2, 0.01);
    control.addColObject("FrontWall", 0.9, 0, 0.5, 1, 0.01, 1.2, 1);
    control.addColObject("BackWall", -0.4, 0, 0.5, 1, 0.01, 1.2, 1);
    control.addColObject("LeftWall", 0.25, 0.6, 0.5, 1, 1.3, 0.01, 1);
    control.addColObject("RightWall", 0.25, -0.6, 0.5, 1, 1.3, 0.01, 1);
    control.addColObject("Camera", 0.65, 0.0, 0.65 + 0.175, 1, 0.1, 0.1, 0.35);
    control.addColObject("TopWall", 0.25, 0, 1.0, 1, 1.28, 1.18, 0.01);

    tf2::Quaternion quat;
    geometry_msgs::Pose start = control.getCurrentPose();

    control.openGripper(start);

    control.print_current_pose_orientation();

    quat[1] = start.orientation.x;
    quat[2] = start.orientation.y;
    quat[3] = start.orientation.z;
    quat[0] = start.orientation.w;

    geometry_msgs::Pose start1 = control.plan_in_xyzw(0.5, 0.02, 0.05, quat, start, 1, 18);

    geometry_msgs::Pose start3 = control.plan_in_xyzw(start1.position.x + 0.12, start1.position.y, start1.position.z, quat, start1, 1, 10);
    control.closeGripper(start3);
    geometry_msgs::Pose start4 = control.plan_in_xyzw(start1.position.x, start1.position.y, start1.position.z, quat, start3, 1, 10);
    geometry_msgs::Pose start5 = control.plan_in_xyzw(start.position.x, start.position.y, start.position.z, quat, start4, 1, 20);
    control.deleteAllObjects();
}
