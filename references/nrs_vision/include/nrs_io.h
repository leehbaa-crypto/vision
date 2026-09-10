#ifndef NRS_IO_H
#define NRS_IO_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <filesystem>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

#include <CGAL/Point_set_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/write_points.h>

#include "nrs_math.h"

namespace fs = boost::filesystem;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Point_set_3<Point_3> Point_set;

class nrs_io
{
public:
    nrs_math n_math;
    std::string pcd_file_path, fMc_file_path, jointState_file_path;
    int pcd_file_counter = 0;
    int fMc_file_counter = 0;


    void loadYAML(const std::string &filename, float &thetaX, float &thetaY, float &thetaZ,
                  float &X, float &Y, float &Z, int &n,
                  float minrange[4], float maxrange[4],
                  float &downsampleparam, int &sor_mean, double &sor_thresh,
                  double &ror_radius, int &ror_neighbor, float &smoothing_radius,
                  float &relative_alpha, float &relative_offset,
                  int &bilateral_k, double &bilateral_sharpness_angle, int &bilateral_iter_number);

    // 포인트 클라우드 메시지를 PCL 포맷으로 변환 후 PCD 파일로 저장하는 함수
    bool savePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    bool savefMcMatrix(const std_msgs::Float64MultiArray::ConstPtr &msg);

    std::vector<Eigen::Matrix4d> loadfMcMatrix(const std::string &registration_input_fMc_path);
    // load point cloud
    std::vector<PointCloud::Ptr> load_PCD_Data(const std::string &file_paths);

    // load point cloud and return Point_set to use in meshing
    Point_set load_Point_set_Data(const std::string &fname);
};

#endif // NRS_IO_H
