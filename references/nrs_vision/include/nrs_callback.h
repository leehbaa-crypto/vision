#ifndef NRS_CALLBACK_H
#define NRS_CALLBACK_H
#include "nrs_io.h"
#include "nrs_pointcloud.h"
#include "nrs_mesh.h"
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <ros/package.h>

class nrs_callback
{
public:
  nrs_callback();
  nrs_io n_io;
  nrs_pointcloud n_pcl;
  nrs_mesh n_mesh;

  /*-------------------------------reconstruction-------------------------------*/

  std::string registration_input_pcd_path, registration_input_fMc_path, registration_output_path, registration_yaml_path;
  std::string reconstruction_input_path, reconstruction_output_path_1, reconstruction_output_path_2, reconstruction_yaml_path;

  float thetaX, thetaY, thetaZ, X, Y, Z;
  float downsampleparam;
  double sor_thresh, ror_radius;
  float minrange[4], maxrange[4], smoothing_radius;


  int n, sor_mean, ror_neighbor;

  int bilateral_k, bilateral_iter_number;
  double bilateral_sharpness_angle;
  float relative_alpha, relative_offset;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  std::vector<Eigen::Matrix4d> matrices;
  

  Point_set points;
  YAML::Node doc;

  /*-------------------------------path simulation-------------------------------*/
  bool scanServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool registrationServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool reconstructionServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

#endif // NRS_CALLBACK_H
