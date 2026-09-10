#include "nrs_pointcloud.h"

namespace fs = boost::filesystem;

pcl::PointCloud<pcl::PointNormal> nrs_pointcloud::smoothing(const PointCloud::Ptr cloud_src, const float &smoothing_radius)
{
    // KD-Tree 생성
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // 결과를 저장할 PointNormal 타입 클라우드
    pcl::PointCloud<pcl::PointNormal> cloud_tgt;

    // Moving Least Squares 객체 초기화 (노말 계산 포함)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud_src);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(smoothing_radius);

    // 처리 수행
    mls.process(cloud_tgt);

    return cloud_tgt;
}

PointCloud::Ptr nrs_pointcloud::downsampling(const PointCloud::Ptr cloud_src, const float &downsampleparam)
{
    if (downsampleparam == 0.0)
    {
        return cloud_src;
    }
    PointCloud::Ptr cloud_tgt(new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud_src);
    vox.setLeafSize(downsampleparam, downsampleparam, downsampleparam);
    vox.filter(*cloud_tgt);
    return cloud_tgt;
}

PointCloud::Ptr nrs_pointcloud::statistical_outlier_remove(const PointCloud::Ptr cloud_src, const int &sor_mean, const double &sor_thresh)
{
    PointCloud::Ptr cloud_tgt(new PointCloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_src);
    sor.setMeanK(sor_mean);
    sor.setStddevMulThresh(sor_thresh);
    sor.filter(*cloud_tgt);
    return cloud_tgt;
}

PointCloud::Ptr nrs_pointcloud::radius_outlier_remove(const PointCloud::Ptr cloud_src, const double &ror_radius, const int &ror_neighbor)
{
    PointCloud::Ptr cloud_filtered(new PointCloud);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud_src);
    ror.setRadiusSearch(ror_radius);
    ror.setMinNeighborsInRadius(ror_neighbor);
    ror.setKeepOrganized(true);
    ror.filter(*cloud_filtered);
    return cloud_filtered;
}

PointCloud::Ptr nrs_pointcloud::calibrate(const PointCloud::Ptr cloud_src, const Eigen::Matrix4f base2tcp, const Eigen::Matrix4f tcp2cam)
{
    PointCloud::Ptr cloud_tcp2cam(new PointCloud);
    PointCloud::Ptr cloud_base2cam(new PointCloud);
    PointCloud::Ptr cloud_marker2cam(new PointCloud);
    // 고정된 marker2base 변환행렬 (charuco marker 기준)
    Eigen::Matrix4f marker2base;
    marker2base << 0.696142, 0.717802, 0.0111144, 0.0541231,
        -0.71635, 0.695145, -0.043272, 0.60022,
        -0.0389855, 0.0221005, 0.998364, 0.00557716,
        0, 0, 0, 1;
    // pcl::transformPointCloud(*cloud_src, *cloud_tcp2cam, tcp2cam);
    pcl::transformPointCloud(*cloud_src, *cloud_base2cam, base2tcp);
    pcl::transformPointCloud(*cloud_base2cam, *cloud_marker2cam, marker2base);
    return cloud_marker2cam;
}

PointCloud::Ptr nrs_pointcloud::calibratemarker2base(const PointCloud::Ptr cloud_src)
{
    PointCloud::Ptr cloud_tgt(new PointCloud);
    Eigen::Matrix4f marker2base, base2marker;
    marker2base << 0.696142, 0.717802, 0.0111144, 0.0541231,
        -0.71635, 0.695145, -0.043272, 0.60022,
        -0.0389855, 0.0221005, 0.998364, 0.00557716,
        0, 0, 0, 1;
    base2marker = marker2base.inverse();
    pcl::transformPointCloud(*cloud_src, *cloud_tgt, base2marker);
    return cloud_tgt;
}

PointCloud::Ptr nrs_pointcloud::charucosegmentation(const PointCloud::Ptr cloud_src, float min_pt[], float max_pt[])
{
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter(true);
    Eigen::Vector4f min_pt_vec4f(min_pt[0], min_pt[1], min_pt[2], min_pt[3]);
    Eigen::Vector4f max_pt_vec4f(max_pt[0], max_pt[1], max_pt[2], max_pt[3]);
    cropBoxFilter.setInputCloud(cloud_src);
    cropBoxFilter.setMin(min_pt_vec4f);
    cropBoxFilter.setMax(max_pt_vec4f);
    PointCloud::Ptr filtered_cloud(new PointCloud);
    cropBoxFilter.filter(*filtered_cloud);
    return filtered_cloud;
}

void nrs_pointcloud::visualizePointClouds(const std::vector<PointCloud::Ptr> &clouds_src)
{
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

    // 밝은 색상 리스트
    std::vector<std::vector<double>> bright_colors = {
        {1.0, 0.0, 0.0}, // Red
        {0.0, 1.0, 0.0}, // Green
        {0.0, 0.0, 1.0}, // Blue
        {1.0, 1.0, 0.0}, // Yellow
        {1.0, 0.0, 1.0}, // Magenta
        {0.0, 1.0, 1.0}, // Cyan
        {1.0, 0.5, 0.0}, // Orange
        {0.0, 1.0, 0.5}, // Lime
        {0.5, 1.0, 0.0}  // Chartreuse
    };

    for (size_t i = 0; i < clouds_src.size(); ++i)
    {
        std::vector<double> color = bright_colors[i % bright_colors.size()];
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(
            clouds_src[i],
            static_cast<int>(color[0] * 255),
            static_cast<int>(color[1] * 255),
            static_cast<int>(color[2] * 255));
        std::string cloud_name = "cloud_" + std::to_string(i);
        viewer.addPointCloud(clouds_src[i], cloud_color_handler, cloud_name);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name);
    }

    viewer.addCoordinateSystem(0.5, "coordinate_system", 0);
    viewer.setPosition(800, 400);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

PointCloud::Ptr nrs_pointcloud::PointNormal2PointXYZ(const pcl::PointCloud<pcl::PointNormal> &cloud_src)
{
    PointCloud::Ptr output_cloud(new PointCloud);
    for (size_t i = 0; i < cloud_src.points.size(); ++i)
    {
        const pcl::PointNormal &mls_pt = cloud_src.points[i];
        PointT pt(mls_pt.x, mls_pt.y, mls_pt.z);
        output_cloud->push_back(pt);
    }
    return output_cloud;
}

PointCloud::Ptr nrs_pointcloud::addPoint(const std::vector<PointCloud::Ptr> &clouds_src)
{
    PointCloud::Ptr result_cloud(new PointCloud);
    for (size_t i = 0; i < clouds_src.size(); ++i)
    {
        *result_cloud += *clouds_src[i];
    }
    return result_cloud;
}

PointCloud::Ptr nrs_pointcloud::euclideanSegmentation(const PointCloud::Ptr cloud_src, double tolerance, int minsize, int maxsize)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_src);

    // 2) EuclideanClusterExtraction 설정 및 실행
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(minsize);
    ec.setMaxClusterSize(maxsize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_src);
    ec.extract(cluster_indices);

    // 클러스터가 없는 경우 빈 클라우드 반환
    if (cluster_indices.empty())
    {
        return PointCloud::Ptr(new PointCloud);
    }

    // 3) 가장 큰 클러스터 인덱스 찾기
    size_t largest_idx = 0;
    size_t max_points = 0;
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        size_t sz = cluster_indices[i].indices.size();
        if (sz > max_points)
        {
            max_points = sz;
            largest_idx = i;
        }
    }

    // 4) 가장 큰 클러스터만 추출
    PointCloud::Ptr largest_cluster(new PointCloud);
    largest_cluster->points.reserve(max_points);
    for (int idx : cluster_indices[largest_idx].indices)
    {
        largest_cluster->points.push_back(cloud_src->points[idx]);
    }
    largest_cluster->width = static_cast<uint32_t>(largest_cluster->points.size());
    largest_cluster->height = 1;
    largest_cluster->is_dense = cloud_src->is_dense;

    return largest_cluster;
}

void nrs_pointcloud::loadAndVisualizePCD(const std::string &filename)
{
    PointCloud::Ptr cloud(new PointCloud);

    if (pcl::io::loadPCDFile<PointT>(filename, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", filename.c_str());
        return;
    }

    std::cout << "Loaded " << cloud->points.size() << " points from " << filename << std::endl;

    pcl::visualization::PCLVisualizer viewer("PCD Viewer");
    viewer.addPointCloud<PointT>(cloud, "sample cloud");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
}

void nrs_pointcloud::saveCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                               const std::string &name, size_t index)
{
    std::string dir = "/home/nrs/catkin_ws/src/nrs_vision/pcd/processed_pcd/";
    std::string filename = dir + name + "_" + std::to_string(index) + ".pcd";
    pcl::io::savePCDFile(filename, *cloud, false);
}

void nrs_pointcloud::registration(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds,
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
                                  const std::string &registration_output_path)
{
    // 전처리 객체 (nrs_pointcloud.h에 정의된 함수들 사용)
    nrs_pointcloud n_pcl;
    nrs_math n_math;

    // 로봇 좌표 변환 관련 변수
    std::vector<Eigen::Matrix4f> base2tcp;                 // 각 클라우드에 대해 계산할 base -> TCP 변환 행렬
    Eigen::Matrix4f tcp2cam = Eigen::Matrix4f::Identity(); // TCP -> Camera 변환 (초기값 항등행렬)

    // 각 단계별 클라우드를 저장할 벡터
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> downsampled_clouds, segmented_clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> calibrated_clouds, radius_outlier_removed_clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> statistical_outlier_removed_clouds, smoothed_clouds, robot_base_calibrated_clouds;

    // smoothing 후 노말 포함 클라우드 및 최종 smoothed 클라우드 (PointXYZ)
    pcl::PointCloud<pcl::PointNormal> smoothed_cloud_withNormal;
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 임시 포인터들
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud, downsampled_cloud, final_cloud;

    // (3) 계산: TCP->Camera 변환 행렬 계산 (로봇의 hand-eye calibration 값 이용)
    // tcp2cam << cos(thetaZ) * cos(thetaY), cos(thetaZ) * sin(thetaY) * sin(thetaX) - sin(thetaZ) * cos(thetaX), cos(thetaZ) * sin(thetaY) * cos(thetaX) + sin(thetaZ) * sin(thetaX), X,
    //     sin(thetaZ) * cos(thetaY), sin(thetaZ) * sin(thetaY) * sin(thetaX) + cos(thetaZ) * cos(thetaX), sin(thetaZ) * sin(thetaY) * cos(thetaX) - cos(thetaZ) * sin(thetaX), Y,
    //     -sin(thetaY), cos(thetaY) * sin(thetaX), cos(thetaY) * cos(thetaX), Z,
    //     0, 0, 0, 1;
    tcp2cam << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // (4) 각 클라우드에 대해 forward kinematics 계산하여 base2tcp 저장
    // (scene 배열의 각 행에 각 클라우드에 해당하는 관절각도가 있다고 가정)
    for (size_t i = 0; i < clouds.size(); ++i)
    {
        // Multiply tcp2cam with matrices[i] (after converting to float) and store in base2tcp.
        // std::cout << matrices[i] << std::endl;
        base2tcp.push_back(matrices[i].cast<float>());
    }

    // (5) 각 클라우드 전처리 파이프라인
    for (size_t i = 0; i < clouds.size(); ++i)
    { // 원본 저장
        saveCloud(clouds[i], "original", i);

        // 다운샘플링
        auto down = n_pcl.downsampling(clouds[i], downsampleparam);
        saveCloud(down, "downsampled", i);
        downsampled_clouds.push_back(down);

        // ROR 필터
        auto ror = n_pcl.radius_outlier_remove(down, ror_radius, ror_neighbor);
        saveCloud(ror, "ror", i);
        radius_outlier_removed_clouds.push_back(ror);

        // Calibration (base->tcp, tcp->cam)
        auto calib = n_pcl.calibrate(ror, base2tcp[i], tcp2cam);
        saveCloud(calib, "calibrated", i);
        calibrated_clouds.push_back(calib);

        // Charuco ROI segmentation
        auto seg = n_pcl.charucosegmentation(calib, minrange, maxrange);
        saveCloud(seg, "segmented", i);
        segmented_clouds.push_back(seg);

        // Marker→Base 보정
        auto markerbase = n_pcl.calibratemarker2base(seg);
        saveCloud(markerbase, "markerbase", i);
        robot_base_calibrated_clouds.push_back(markerbase);

        // 통계적 이상치 제거
        // auto sor = n_pcl.statistical_outlier_remove(markerbase, sor_mean, sor_thresh);
        // saveCloud(sor, "sor", i);
        // statistical_outlier_removed_clouds.push_back(sor);
    }

    // (6) 여러 클라우드를 하나로 합치고, 다시 다운샘플링하여 최종 클라우드 생성
    result_cloud = n_pcl.downsampling(n_pcl.addPoint(robot_base_calibrated_clouds), downsampleparam * 2.5);
    saveCloud(result_cloud, "result", 0);
    auto sor_cloud= n_pcl.statistical_outlier_remove(result_cloud, sor_mean, sor_thresh);
    saveCloud(sor_cloud, "sor_final", 0);
    final_cloud = n_pcl.euclideanSegmentation(sor_cloud, 0.01, 1000, 500000);
    // 유클리드 클러스터 segmentation 결과가 있다면
    saveCloud(final_cloud, "euclidean", 0);
    // (7) 스무딩 처리 (Moving Least Squares를 사용하여 노말 추정 포함)
    smoothed_cloud_withNormal = n_pcl.smoothing(final_cloud, smoothing_radius);
    ROS_INFO("Registration process : Smoothing Completed");
    smoothed_cloud = n_pcl.PointNormal2PointXYZ(smoothed_cloud_withNormal);
    smoothed_clouds.push_back(final_cloud);
    // n_pcl.visualizePointClouds(smoothed_clouds);

    // (8) 최종 결과 저장 (PCD 파일로 저장)
    pcl::io::savePCDFile(registration_output_path, *smoothed_cloud, false);

    // std::this_thread::sleep_for(std::chrono::seconds(5));
    // n_pcl.loadAndVisualizePCD(registration_output_path);
    std::cout << "Final registered point cloud saved to " << registration_output_path << std::endl;
}

PointCloud::Ptr nrs_pointcloud::MinCutSegmentation(const PointCloud::Ptr &cloud_src,
                                                   const PointT &foreground_center,
                                                   double radius = 0.07, double sigma = 0.25, double source_weight = 0.8, int num_neighbors = 14)
{
    // NaN 제거용 인덱스 생성
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud_src, *indices);

    // MinCut Segmentation 객체 설정
    pcl::MinCutSegmentation<PointT> seg;
    seg.setInputCloud(cloud_src);
    seg.setIndices(indices);

    // 전경 포인트 설정
    PointCloud::Ptr foreground_points(new PointCloud);
    foreground_points->push_back(foreground_center);
    seg.setForegroundPoints(foreground_points);

    // 파라미터 설정
    seg.setSigma(sigma);
    seg.setRadius(radius);
    seg.setSourceWeight(source_weight);
    seg.setNumberOfNeighbours(num_neighbors);

    // 클러스터 추출
    std::vector<pcl::PointIndices> clusters;
    seg.extract(clusters);

    // 배경 추출
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_colored_cloud = seg.getColoredCloud();
    PointCloud::Ptr background_cloud(new PointCloud);

    // 빨간색(R=255, G=0, B=0)이 아닌 포인트는 배경으로 간주
    for (const auto &pt : full_colored_cloud->points)
    {
        if ((pt.r == 255 && pt.g == 0 && pt.b == 0)) // foreground가 빨간색
        {
            PointT p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            background_cloud->points.push_back(p);
        }
    }

    return background_cloud;
}