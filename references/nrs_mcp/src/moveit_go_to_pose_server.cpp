#include <ros/ros.h>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <map>
#include <cmath>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/OrientationConstraint.h>

#include <nrs_mcp/GoToPose.h>

class GoToPoseServer
{
public:
  GoToPoseServer(ros::NodeHandle &nh_pub)
      : nh_pub_(nh_pub),
        nh_priv_("~"),
        tf_buffer_(ros::Duration(10.0)),
        tf_listener_(tf_buffer_)
  {
    // params (private namespace)
    group_name_ = nh_priv_.param<std::string>("group_name", "manipulator");
    eef_link_ = nh_priv_.param<std::string>("eef_link", "tool0");
    ref_frame_ = nh_priv_.param<std::string>("reference_frame", "base_link");

    pos_tol_ = nh_priv_.param("goal_position_tolerance", 0.005);   // m
    ori_tol_ = nh_priv_.param("goal_orientation_tolerance", 0.05); // rad
    min_fraction_ = nh_priv_.param("cartesian_min_fraction", 0.80);

    planner_id_ = nh_priv_.param<std::string>("planner_id", "RRTConnectkConfigDefault");
    attempts_ = nh_priv_.param("num_planning_attempts", 20);
    plan_time_ = nh_priv_.param("planning_time", 5.0);

    use_pos_only_ = nh_priv_.param("position_only_ik", false);
    use_ws_ = nh_priv_.param("use_workspace", false);
    ws_min_x_ = nh_priv_.param("workspace/min_x", -0.8);
    ws_min_y_ = nh_priv_.param("workspace/min_y", -0.8);
    ws_min_z_ = nh_priv_.param("workspace/min_z", 0.0);
    ws_max_x_ = nh_priv_.param("workspace/max_x", 0.8);
    ws_max_y_ = nh_priv_.param("workspace/max_y", 0.8);
    ws_max_z_ = nh_priv_.param("workspace/max_z", 1.4);

    lock_ori_ = nh_priv_.param("lock_orientation", false);
    lock_ori_tol_ = nh_priv_.param("lock_ori_tolerance", 0.2); // rad

    // MoveGroup 준비 (기존 코드)
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(group_name_);
    move_group_->setEndEffectorLink(eef_link_);
    move_group_->setPoseReferenceFrame(ref_frame_);
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);

    move_group_->setGoalPositionTolerance(pos_tol_);
    move_group_->setGoalOrientationTolerance(ori_tol_);
    move_group_->setPlannerId(planner_id_);
    move_group_->setNumPlanningAttempts(attempts_);
    move_group_->setPlanningTime(plan_time_);
    if (use_ws_)
    {
      move_group_->setWorkspace(ws_min_x_, ws_min_y_, ws_min_z_,
                                ws_max_x_, ws_max_y_, ws_max_z_);
    }

    if (use_pos_only_)
    {
      ROS_INFO("Position-only IK expected. Ensure /robot_description_kinematics/manipulator/position_only_ik=true");
    }

    /* ====== 여기부터 추가: 초기 자세로 이동 ====== */
    bool go_initial = nh_priv_.param("go_initial_on_start", true); // 파라미터로 토글 가능
    double settle_sec = nh_priv_.param("initial_settle_sec", 0.5); // 약간 대기

    if (go_initial)
    {
      ROS_INFO("[go_to_pose_server] Moving to initial joint pose...");

      std::map<std::string, double> initial_pose = {
          {"shoulder_pan_joint", -30.0 * M_PI / 180.0},
          {"shoulder_lift_joint", -55.54 * M_PI / 180.0},
          {"elbow_joint", -107.51 * M_PI / 180.0},
          {"wrist_1_joint", -107.45 * M_PI / 180.0},
          {"wrist_2_joint", 92.18 * M_PI / 180.0},
          {"wrist_3_joint", 20.74 * M_PI / 180.0},
      };

      move_group_->setStartStateToCurrentState();
      move_group_->setJointValueTarget(initial_pose);

      // move()는 내부적으로 plan+execute 수행
      auto exec_ret = move_group_->move();
      if (exec_ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        ROS_WARN("[go_to_pose_server] Initial move failed (code=%d). Continuing anyway.", exec_ret.val);
      }
      else
      {
        ROS_INFO("[go_to_pose_server] Initial move done.");
      }

      // 약간 안정화 시간
      if (settle_sec > 0.0)
      {
        ros::Duration(settle_sec).sleep();
      }

      // 이후 요청 시 시작상태 최신화가 의미 있도록 한번 더 맞춰둠
      move_group_->setStartStateToCurrentState();
    }
    /* ====== 초기 자세 이동 끝 ====== */

    // 퍼블릭 서비스로 광고 → /go_to_pose
    service_ = nh_pub_.advertiseService("/go_to_pose", &GoToPoseServer::handleService, this);

    ROS_INFO_STREAM("[nrs_mcp/go_to_pose] ready. group=" << group_name_
                                                         << " eef=" << eef_link_
                                                         << " ref=" << ref_frame_
                                                         << " pos_tol=" << pos_tol_
                                                         << " ori_tol=" << ori_tol_
                                                         << " min_frac=" << min_fraction_
                                                         << " planner=" << planner_id_
                                                         << " attempts=" << attempts_
                                                         << " time=" << plan_time_);
  }

private:
  bool handleService(nrs_mcp::GoToPose::Request &req,
                     nrs_mcp::GoToPose::Response &res)
  {
    try
    {
      ROS_INFO("==== [GoToPose Request] ====");
      ROS_INFO("req.target_frame = %s", req.target_frame.c_str());
      ROS_INFO("req.pose.position = [%.3f, %.3f, %.3f]",
               req.pose.position.x, req.pose.position.y, req.pose.position.z);
      ROS_INFO("req.pose.orientation = [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
               req.pose.orientation.x, req.pose.orientation.y,
               req.pose.orientation.z, req.pose.orientation.w);

      move_group_->setMaxVelocityScalingFactor(std::clamp(static_cast<double>(req.vel_scale), 0.0, 1.0));
      move_group_->setMaxAccelerationScalingFactor(std::clamp(static_cast<double>(req.acc_scale), 0.0, 1.0));
      move_group_->setPlanningTime(std::max(plan_time_, static_cast<double>(req.allowed_planning_time)));
      move_group_->setStartStateToCurrentState();
      move_group_->clearPathConstraints();

      const std::string planning_frame = move_group_->getPoseReferenceFrame();
      geometry_msgs::PoseStamped target_ps = transformPose(req.pose, req.target_frame, planning_frame);

      if (target_ps.header.frame_id.empty())
      {
        res.success = false;
        res.message = "TF transform failed";
        return true;
      }

      ROS_INFO("Target pose in planning_frame='%s'", planning_frame.c_str());
      ROS_INFO("target_ps.position = [%.3f, %.3f, %.3f]",
               target_ps.pose.position.x, target_ps.pose.position.y, target_ps.pose.position.z);
      ROS_INFO("target_ps.orientation = [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
               target_ps.pose.orientation.x, target_ps.pose.orientation.y,
               target_ps.pose.orientation.z, target_ps.pose.orientation.w);

      if (req.cartesian)
      {
        ROS_INFO("Planning Cartesian path...");
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(move_group_->getCurrentPose(eef_link_).pose);
        waypoints.push_back(target_ps.pose);

        moveit_msgs::RobotTrajectory traj_msg;
        double fraction = move_group_->computeCartesianPath(
            waypoints,
            req.eef_step > 0.0 ? req.eef_step : 0.005,
            req.jump_threshold,
            traj_msg,
            true);

        ROS_INFO("Cartesian planning done. fraction=%.3f", fraction);

        if (req.plan_only)
        {
          bool ok = (!traj_msg.joint_trajectory.points.empty() && fraction >= min_fraction_);
          res.success = ok;
          res.message = ok ? "planned cartesian" : "planning failed";
          return true;
        }
        if (traj_msg.joint_trajectory.points.empty() || fraction < min_fraction_)
        {
          res.success = false;
          res.message = "cartesian planning failed";
          return true;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj_msg;
        ROS_INFO("Executing Cartesian path...");
        bool ok = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Execution %s", ok ? "SUCCESS" : "FAILED");
        res.success = ok;
        res.message = ok ? "executed cartesian" : "execution failed";
        return true;
      }
      else
      {
        ROS_INFO("Planning joint-space trajectory...");
        move_group_->setPoseTarget(target_ps, eef_link_);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto code = move_group_->plan(plan);
        int code_val = code.val; // MoveItErrorCode → int
        ROS_INFO("Planning result code=%d, points=%zu",
                 code_val, plan.trajectory_.joint_trajectory.points.size());

        if (req.plan_only)
        {
          move_group_->clearPoseTargets();
          bool ok = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS &&
                     !plan.trajectory_.joint_trajectory.points.empty());
          res.success = ok;
          res.message = ok ? "planned joint trajectory" : "planning failed";
          return true;
        }
        if (code != moveit::planning_interface::MoveItErrorCode::SUCCESS ||
            plan.trajectory_.joint_trajectory.points.empty())
        {
          move_group_->clearPoseTargets();
          res.success = false;
          res.message = "planning failed";
          return true;
        }
        ROS_INFO("Executing trajectory...");
        bool ok = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Execution %s", ok ? "SUCCESS" : "FAILED");
        move_group_->stop();
        move_group_->clearPoseTargets();

        res.success = ok;
        res.message = ok ? "executed trajectory" : "execution failed";
        return true;
      }
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM("go_to_pose error: " << e.what());
      res.success = false;
      res.message = std::string("error: ") + e.what();
      return true;
    }
  }

  geometry_msgs::PoseStamped transformPose(const geometry_msgs::Pose &pose_in,
                                           const std::string &from_frame,
                                           const std::string &to_frame)
  {
    geometry_msgs::PoseStamped in, out;
    in.header.stamp = ros::Time(0);
    in.header.frame_id = from_frame;
    in.pose = pose_in;

    if (from_frame == to_frame)
      return in;

    try
    {
      geometry_msgs::TransformStamped tf =
          tf_buffer_.lookupTransform(to_frame, from_frame, ros::Time(0), ros::Duration(1.0));
      tf2::doTransform(in, out, tf);
      return out;
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM("TF transform failed " << from_frame << " -> " << to_frame << ": " << e.what());
      out.header.frame_id.clear();
      return out;
    }
  }

private:
  ros::NodeHandle nh_pub_;
  ros::NodeHandle nh_priv_;
  ros::ServiceServer service_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // params
  std::string group_name_, eef_link_, ref_frame_;
  double pos_tol_, ori_tol_, min_fraction_;
  std::string planner_id_;
  int attempts_;
  double plan_time_;
  bool use_pos_only_;
  bool use_ws_;
  double ws_min_x_, ws_min_y_, ws_min_z_, ws_max_x_, ws_max_y_, ws_max_z_;
  bool lock_ori_;
  double lock_ori_tol_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_go_to_pose_server");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh_pub; // public
  GoToPoseServer server(nh_pub);

  ros::waitForShutdown();
  return 0;
}
