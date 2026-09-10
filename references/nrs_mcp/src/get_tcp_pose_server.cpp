#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>

#include <nrs_mcp/GetTcpPose.h>

class TcpPoseServer {
public:
  TcpPoseServer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : buffer_(ros::Duration(5.0)), listener_(buffer_)
  {
    pnh.param<std::string>("base_frame", base_frame_, "base");
    pnh.param<std::string>("tool_frame", tool_frame_, "tool0");
    pnh.param<std::string>("camera_frame", camera_frame_, "camera_link");
    pnh.param<double>("publish_rate", publish_rate_, 30.0);
    pnh.param<double>("lookup_timeout", lookup_timeout_, 0.2);

    pub_tcp_pose_ = nh.advertise<geometry_msgs::PoseStamped>("tcp_pose", 10);
    pub_fMc_      = nh.advertise<std_msgs::Float64MultiArray>("fMc_matrix", 10);

    srv_ = nh.advertiseService("get_tcp_pose", &TcpPoseServer::handleGetTcpPose, this);

    ROS_INFO("get_tcp_pose ready. base=%s, tool=%s, cam=%s",
             base_frame_.c_str(), tool_frame_.c_str(), camera_frame_.c_str());
  }

  void spin() {
    ros::Rate rate(publish_rate_);
    while (ros::ok()) {
      publishOnce();
      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  bool handleGetTcpPose(nrs_mcp::GetTcpPose::Request&,
                        nrs_mcp::GetTcpPose::Response& res)
  {
    geometry_msgs::TransformStamped tf;
    if (lookup(base_frame_, tool_frame_, tf)) {
      res.pose = toPoseStamped(tf);
    } else {
      res.pose.header.stamp = ros::Time::now();
      res.pose.header.frame_id = base_frame_;
    }
    return true;
  }

  void publishOnce() {
    // 1) TCP pose publish
    geometry_msgs::TransformStamped tf_bt;
    if (lookup(base_frame_, tool_frame_, tf_bt)) {
      pub_tcp_pose_.publish(toPoseStamped(tf_bt));
    }

    // 2) fMc matrix publish (base -> camera_link)
    geometry_msgs::TransformStamped tf_bc;
    if (lookup(base_frame_, camera_frame_, tf_bc)) {
      Eigen::Isometry3d Tbc = tf2::transformToEigen(tf_bc);
      std_msgs::Float64MultiArray msg;
      fillMatrixMsgRowMajor(Tbc.matrix(), msg);
      pub_fMc_.publish(msg);
    }
  }

  bool lookup(const std::string& src,
              const std::string& tgt,
              geometry_msgs::TransformStamped& out)
  {
    try {
      out = buffer_.lookupTransform(src, tgt, ros::Time(0),
                                    ros::Duration(lookup_timeout_));
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "TF lookup failed %s -> %s: %s",
                        src.c_str(), tgt.c_str(), ex.what());
      return false;
    }
  }

  static geometry_msgs::PoseStamped toPoseStamped(const geometry_msgs::TransformStamped& t) {
    geometry_msgs::PoseStamped ps;
    ps.header = t.header;
    ps.pose.position.x = t.transform.translation.x;
    ps.pose.position.y = t.transform.translation.y;
    ps.pose.position.z = t.transform.translation.z;
    ps.pose.orientation = t.transform.rotation;
    return ps;
  }

  static void fillMatrixMsgRowMajor(const Eigen::Matrix4d& M, std_msgs::Float64MultiArray& out) {
    out.layout.dim.resize(2);
    out.layout.dim[0].label = "rows";
    out.layout.dim[0].size  = 4;
    out.layout.dim[0].stride= 16;
    out.layout.dim[1].label = "cols";
    out.layout.dim[1].size  = 4;
    out.layout.dim[1].stride= 4;
    out.data.resize(16);
    // row-major flatten
    int k = 0;
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        out.data[k++] = M(r, c);
      }
    }
  }

  std::string base_frame_, tool_frame_, camera_frame_;
  double publish_rate_, lookup_timeout_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  ros::Publisher pub_tcp_pose_;
  ros::Publisher pub_fMc_;
  ros::ServiceServer srv_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "get_tcp_pose_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  TcpPoseServer node(nh, pnh);
  node.spin();
  return 0;
}
