#include "Planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Arc");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // setup arm joints and the scene

    ArmControl control("arm", "arm_link0");
    control.print_current_pose_position();
    control.print_current_pose_orientation();

    // control.publishSphere();

    geometry_msgs::Pose c = control.getCurrentPose();

    float deg = 90;
    float r = 0.2365;
    std::vector<geometry_msgs::Pose> points;
    points.resize(deg + 1);
    points[0] = control.getCurrentPose();
    for (int i = 0; i < deg; i++)
    {
        points[i + 1] = points[0];
        points[i + 1].position.y = points[0].position.y - r * sin(i * PI / 180);
        points[i + 1].position.x = points[0].position.x + (r - r * cos(i * PI / 180));
        /*
        ROS_INFO_NAMED("Current_Pose_Orientation", "x: %f", points[i+1].position.x);
        ROS_INFO_NAMED("Current_Pose_Orientation", "y: %f", points[i+1].position.y);
        */
    }

    std::vector<geometry_msgs::Pose> pointsB;
    pointsB.resize(deg + 1);
    pointsB[0] = points.back();
    tf2::Quaternion normal;
    normal.setRPY(0, 0, 0);
    for (int i = 0; i < deg; i++)
    {
        tf2::Quaternion quat;

        quat.setRPY(0, 0, i * PI / 180);
        pointsB[i + 1] = pointsB[0];
        pointsB[i + 1].position.y = pointsB[0].position.y + (r - r * cos(i * PI / 180));
        pointsB[i + 1].position.x = pointsB[0].position.x - r * sin(i * PI / 180);
        pointsB[i + 1].orientation.x += (normal[0] - quat[0]);
        pointsB[i + 1].orientation.y += (normal[1] - quat[1]);
        pointsB[i + 1].orientation.z += (normal[2] - quat[2]);
        pointsB[i + 1].orientation.w += (normal[3] - quat[3]);
        /*
        ROS_INFO_NAMED("Current_Pose_Orientation", "x: %f", pointsB[i+1].position.x);
        ROS_INFO_NAMED("Current_Pose_Orientation", "y: %f", pointsB[i+1].position.y);
        */
        points.push_back(pointsB[i + 1]);
    }

    float result = 0;
    int count = 0;
    while (result != 1)
    {

        result = control.plan_cartesian_path(points, 0, 0);
        ROS_INFO_NAMED("Planning", "Attempt: %d  planned: %f", count, result);
        count++;
    }
}