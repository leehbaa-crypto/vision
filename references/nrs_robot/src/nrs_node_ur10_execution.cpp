#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

struct Waypoint
{
    double x, y, z;
};

std::vector<Waypoint> loadWaypoints(const std::string &path)
{
    std::vector<Waypoint> waypoints;
    std::ifstream file(path);
    std::string line;

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::vector<double> vals;
        double val;

        while (ss >> val)
            vals.push_back(val);

        if (vals.size() >= 3)
        {
            waypoints.push_back({vals[0], vals[1], vals[2]});
        }
        else
        {
            ROS_WARN_STREAM("Skipping invalid line: [" << line << "]");
        }
    }

    return waypoints;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nrs_node_ur10_cartesian");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
       moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);

    // tooltip이 따라가야 하는 orientation: z- 방향 (Rx=π, Ry=0, Rz=-2.3)
    tf2::Quaternion q_tip;
    q_tip.setRPY(M_PI, 0.0, -2.3);
    geometry_msgs::Quaternion tooltip_orientation;
    tooltip_orientation = tf2::toMsg(q_tip);

    // Tooltip offset (TCP 기준으로 z축 15cm 앞으로 나감)
    tf2::Vector3 offset_tcp(0.0, 0.0, 0.14); // 15cm

    // Waypoint 파일 경로
    std::string path = "/home/nrs/catkin_ws/src/nrs_path/data/interpolated_continuous_waypoints.txt";
    std::vector<Waypoint> raw_waypoints = loadWaypoints(path);
    ROS_INFO_STREAM("Loaded " << raw_waypoints.size() << " waypoints.");

    if (raw_waypoints.empty())
    {
        ROS_ERROR("No waypoints loaded. Exiting.");
        return 1;
    }

    std::map<std::string, double> initial_pose = {
        {"shoulder_pan_joint", 10.95 * M_PI / 180},
        {"shoulder_lift_joint", -73.33 * M_PI / 180},
        {"elbow_joint", -116.49 * M_PI / 180},
        {"wrist_1_joint", -80.22 * M_PI / 180},
        {"wrist_2_joint", 89.60 * M_PI / 180},
        {"wrist_3_joint", 52.86 * M_PI / 180}};

    move_group.setJointValueTarget(initial_pose);
    move_group.move();

    // Tooltip → EE 역산된 pose 계산
    std::vector<geometry_msgs::Pose> ee_poses;
    for (const auto &wp : raw_waypoints)
    {
        // Tooltip pose
        tf2::Transform T_tooltip;
        T_tooltip.setOrigin(tf2::Vector3(wp.x, wp.y, wp.z));
        tf2::Quaternion q_tip;
        tf2::fromMsg(tooltip_orientation, q_tip);
        T_tooltip.setRotation(q_tip);

        // EE pose = tooltip ⊖ offset
        tf2::Transform T_offset;
        T_offset.setOrigin(offset_tcp);
        T_offset.setRotation(tf2::Quaternion(0, 0, 0, 1));
        tf2::Transform T_ee = T_tooltip * T_offset.inverse();

        geometry_msgs::Pose ee_pose;
        ee_pose.position.x = T_ee.getOrigin().x();
        ee_pose.position.y = T_ee.getOrigin().y();
        ee_pose.position.z = T_ee.getOrigin().z();
        ee_pose.orientation = tf2::toMsg(T_ee.getRotation());

        ee_poses.push_back(ee_pose);
    }

    // Cartesian Path 계획
    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.005;     // 0.5cm resolution
    const double jump_threshold = 0.0; // disable jump prevention

    double fraction = move_group.computeCartesianPath(
        ee_poses, eef_step, jump_threshold, trajectory);

    ROS_INFO_STREAM("Cartesian path planning completed: " << 100.0 * fraction << "%");

    if (fraction > 0.9)
    {
        // 실행
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group.execute(plan);
        ROS_INFO("Trajectory execution completed.");
    }
    else
    {
        ROS_WARN("Planned path is insufficient. Not executing.");
    }

    move_group.setJointValueTarget(initial_pose);
    move_group.move();

    ros::shutdown();
    return 0;
}
