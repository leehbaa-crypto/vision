#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_publisher");
  ros::NodeHandle nh("~");

  std::string pcd_path, topic, frame_id;
  nh.param<std::string>("pcd_path", pcd_path, std::string(""));
  nh.param<std::string>("topic", topic, std::string("/pcd_cloud"));
  nh.param<std::string>("frame_id", frame_id, std::string("map"));

  if (pcd_path.empty()) {
    ROS_ERROR("pcd_path param is empty. set ~pcd_path:=/path/to/file.pcd");
    return 1;
  }

  // latch = true 로 advertise → 한 번만 publish해도 RViz가 바로 수신
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic, 1, true);

  pcl::PCLPointCloud2 cloud_blob;
  if (pcl::io::loadPCDFile(pcd_path, cloud_blob) < 0) {
    ROS_ERROR_STREAM("Failed to read PCD: " << pcd_path);
    return 1;
  }
  ROS_INFO_STREAM("Loaded PCD: " << pcd_path
                  << "  points: " << cloud_blob.width * cloud_blob.height
                  << "  fields: " << cloud_blob.fields.size());

  sensor_msgs::PointCloud2 msg;
  pcl_conversions::moveFromPCL(cloud_blob, msg);
  msg.header.frame_id = frame_id;
  msg.header.stamp = ros::Time::now();

  // 한 번 퍼블리시
  ros::Duration(0.5).sleep(); // RViz가 붙을 시간 약간 주기
  pub.publish(msg);
  ROS_INFO_STREAM("Published to " << topic << " with frame_id=" << frame_id);

  ros::spin();  // 노드 유지(라치 메시지 유지용)
  return 0;
}
