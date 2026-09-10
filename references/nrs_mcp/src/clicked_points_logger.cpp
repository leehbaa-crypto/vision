#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <vector>
#include <string>
#include <fstream>
#include <iomanip>

class ClickedPointsLogger
{
public:
  ClickedPointsLogger(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  {
    // params
    pnh_.param<std::string>("input_topic", input_topic_, std::string("/clicked_point"));
    pnh_.param<std::string>("marker_topic", marker_topic_, std::string("/visualization_marker"));
    pnh_.param<std::string>("frame_id", frame_id_, std::string("map"));
    pnh_.param<std::string>("waypoint_file", waypoint_file_, std::string("selected_waypoints.txt"));
    pnh_.param<double>("marker_scale", marker_scale_, 0.005); // meters
    pnh_.param<bool>("clear_file_on_start", clear_on_start_, true);

    // publishers / subscribers
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 10, false);
    sub_ = nh_.subscribe(input_topic_, 100, &ClickedPointsLogger::cbPoint, this);

    // start fresh file (truncate)
    if (clear_on_start_) {
      writeAllToFile(); // empty list -> file cleared
      ROS_INFO_STREAM("Truncated file at start: " << waypoint_file_);
    }

    ROS_INFO_STREAM("Listening: " << input_topic_);
    ROS_INFO_STREAM("Publishing markers on: " << marker_topic_ << " with frame_id=" << frame_id_);
    ROS_INFO_STREAM("Writing waypoints to: " << waypoint_file_);
  }

private:
  void cbPoint(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    // store point (use the point values as-is)
    points_.push_back(msg->point);

    // publish marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "clicked_points";
    marker.id = next_id_++;  // unique id per point
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position = msg->point;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = marker_scale_;
    marker.scale.y = marker_scale_;
    marker.scale.z = marker_scale_;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration(0.0); // forever

    marker_pub_.publish(marker);

    // rewrite file with all points collected so far
    if (!writeAllToFile()) {
      ROS_WARN_STREAM("Failed to write waypoints to " << waypoint_file_);
    } else {
      ROS_INFO_STREAM("Saved " << points_.size() << " waypoint(s) to " << waypoint_file_);
    }
  }

  bool writeAllToFile()
  {
    std::ofstream ofs(waypoint_file_, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) return false;

    ofs << std::fixed << std::setprecision(6);
    // 간단한 헤더 줄(원하면 지워도 됨)
    ofs << "# x y z\n";
    for (const auto& p : points_) {
      ofs << p.x << " " << p.y << " " << p.z << "\n";
    }
    ofs.close();
    return true;
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher marker_pub_;

  std::string input_topic_;
  std::string marker_topic_;
  std::string frame_id_;
  std::string waypoint_file_;
  double marker_scale_;
  bool clear_on_start_ {true};

  std::vector<geometry_msgs::Point> points_;
  int next_id_ {0};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clicked_points_logger");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ClickedPointsLogger node(nh, pnh);
  ros::spin();
  return 0;
}
