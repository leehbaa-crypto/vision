#ifndef NRS_POINTCLOUD_H
#define NRS_POINTCLOUD_H

#include "nrs_math.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/filter_indices.h>

#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <thread>    // 추가
#include <chrono>    // 추가
// 타입 별칭 정의
typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<PointT> PointCloud;

class nrs_pointcloud
{
public:
    // Moving Least Squares smoothing (노말 계산 포함)
    pcl::PointCloud<pcl::PointNormal> smoothing(const PointCloud::Ptr cloud_src, const float &smoothing_radius);

    // VoxelGrid 필터를 이용한 다운샘플링
    PointCloud::Ptr downsampling(const PointCloud::Ptr cloud_src, const float &downsampleparam);

    // Statistical Outlier Removal 필터
    PointCloud::Ptr statistical_outlier_remove(const PointCloud::Ptr cloud_src, const int &sor_mean, const double &sor_thresh);

    // Radius Outlier Removal 필터
    PointCloud::Ptr radius_outlier_remove(const PointCloud::Ptr cloud_src, const double &ror_radius, const int &ror_neighbor);

    // 캘리브레이션: base2tcp, tcp2cam 변환행렬을 이용하여 포인트 클라우드 변환
    PointCloud::Ptr calibrate(const PointCloud::Ptr cloud_src, const Eigen::Matrix4f base2tcp, const Eigen::Matrix4f tcp2cam);

    // 고정된 마커 기준 캘리브레이션 (marker2base)
    PointCloud::Ptr calibratemarker2base(const PointCloud::Ptr cloud_src);

    // Charuco segmentation: CropBox 필터를 이용하여 지정 영역 추출 (min_pt, max_pt는 길이 4의 배열)
    PointCloud::Ptr charucosegmentation(const PointCloud::Ptr cloud_src, float min_pt[], float max_pt[]);

    // PCLVisualizer를 이용해 여러 포인트 클라우드를 시각화
    void visualizePointClouds(const std::vector<PointCloud::Ptr> &clouds_src);

    // pcl::PointNormal 타입 포인트 클라우드를 PointXYZ 타입으로 변환
    PointCloud::Ptr PointNormal2PointXYZ(const pcl::PointCloud<pcl::PointNormal> &cloud_src);

    // 여러 포인트 클라우드를 합산하여 하나의 클라우드로 생성
    PointCloud::Ptr addPoint(const std::vector<PointCloud::Ptr> &clouds_src);

    // 가장 point 개수가 많은 pointcloud만 추출
    PointCloud::Ptr euclideanSegmentation(const PointCloud::Ptr cloud_src, double tolerance, int minsize, int maxsize);

    void loadAndVisualizePCD(const std::string &filename);
    void saveCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,const std::string& name, size_t index);

    void registration(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds,
                      std::vector<Eigen::Matrix4d> &matrices,
                      float thetaX, float thetaY, float thetaZ,
                      float X, float Y, float Z,
                      int n,
                      float minrange[4],
                      float maxrange[4],
                      float downsampleparam,
                      int sor_mean,
                      double sor_thresh,
                      double ror_radius,
                      int ror_neighbor,
                      float smoothing_radius,
                      float relative_alpha,
                      float relative_offset,
                      int bilateral_k,
                      double bilateral_sharpness_angle,
                      int bilateral_iter_number,
                      const std::string &registration_output_path);

    PointCloud::Ptr MinCutSegmentation(const PointCloud::Ptr &cloud_src,
                                       const PointT &foreground_center,
                                       double radius, double sigma, double source_weight, int num_neighbors);
};

#endif // NRS_POINTCLOUD_H
