#include "nrs_callback.h"

nrs_callback::nrs_callback()
{
}


bool nrs_callback::scanServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Scan service called. Waiting for point cloud...");

    // 최대 10초 동안 "/camera/depth/points" 토픽의 포인트 클라우드 메시지를 기다립니다.
    sensor_msgs::PointCloud2ConstPtr cloud_msg =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/points", ros::Duration(10.0));

    if (!cloud_msg)
    {
        ROS_ERROR("Timeout waiting for point cloud.");
        return false;
    }

    // 2. fMc matrix 대기
    std_msgs::Float64MultiArrayConstPtr fMc_msg =
        ros::topic::waitForMessage<std_msgs::Float64MultiArray>("fMc_matrix", ros::Duration(10.0));

    if (!fMc_msg)
    {
        ROS_ERROR("Timeout waiting for fMc matrix.");
        return false;
    }

    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    PointCloud::Ptr downsampled_cloud = n_pcl.downsampling(cloud, 0.002);
    // PointT centerpoint;
    // centerpoint.x = 0.1;
    // centerpoint.y = 0.0;
    // centerpoint.z = 0.45;
    // ROS_INFO("segmenting point cloud...");
    // PointCloud::Ptr segmented_cloud =
    //     n_pcl.MinCutSegmentation(downsampled_cloud,
    //                              centerpoint,
    //                              0.15, 0.02, 1.0, 10);

    return (n_io.savePointCloud(downsampled_cloud) && n_io.savefMcMatrix(fMc_msg));
}

bool nrs_callback::registrationServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Registration service called. Starting registration process...");

    try
    {

        // (1) PCD 파일 로드: nrs_pointcloud의 loadData 함수를 사용하여 폴더 내 모든 PCD 파일을 읽어옴
        clouds = n_io.load_PCD_Data(registration_input_pcd_path);

        matrices = n_io.loadfMcMatrix(registration_input_fMc_path);

        // (2) YAML 파일에서 로봇 및 전처리 파라미터 로드

        // loadYAML은 첫 번째 행(즉, scene[0])부터 n행의 데이터를 채워 넣는다고 가정합니다.
        n_io.loadYAML(registration_yaml_path, thetaX, thetaY, thetaZ, X, Y, Z,
                      n, minrange, maxrange, downsampleparam, sor_mean, sor_thresh,
                      ror_radius, ror_neighbor, smoothing_radius, relative_alpha, relative_offset,
                      bilateral_k, bilateral_sharpness_angle, bilateral_iter_number);
        ROS_INFO("Loading Yaml process completed.");
        // nrs_pointcloud::registration() 함수는 폴더 경로, YAML 파일 경로, 출력 파일 경로를 인자로 받아
        // 전체 등록 파이프라인을 수행하고 최종 PCD 파일을 저장합니다.
        n_pcl.registration(clouds, matrices, thetaX, thetaY, thetaZ, X, Y, Z,
                           n, minrange, maxrange, downsampleparam, sor_mean, sor_thresh,
                           ror_radius, ror_neighbor, smoothing_radius, relative_alpha, relative_offset,
                           bilateral_k, bilateral_sharpness_angle, bilateral_iter_number, registration_output_path);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Registration failed: %s", e.what());
        return false;
    }

    ROS_INFO("Registration process completed. Output saved to %s", registration_output_path.c_str());
    return true;
}

bool nrs_callback::reconstructionServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("reconstruction service called. Starting registration process...");

    try
    {
        nrs_mesh n_mesh;
        points = n_io.load_Point_set_Data(reconstruction_input_path);
        doc = YAML::LoadFile(reconstruction_yaml_path);
        bilateral_k = doc["bilateral_k"][0].as<int>();
        bilateral_sharpness_angle = doc["bilateral_sharpness_angle"][0].as<double>();
        bilateral_iter_number = doc["bilateral_iter_number"][0].as<int>();
        relative_alpha = doc["relative_alpha"][0].as<double>();
        relative_offset = doc["relative_offset"][0].as<double>();

        // nrs_pointcloud::registration() 함수는 폴더 경로, YAML 파일 경로, 출력 파일 경로를 인자로 받아
        // 전체 등록 파이프라인을 수행하고 최종 PCD 파일을 저장합니다.
        n_mesh.reconstruction(points, bilateral_k, bilateral_sharpness_angle, bilateral_iter_number, relative_alpha, relative_offset, reconstruction_output_path_1);
        n_mesh.reconstruction(points, bilateral_k, bilateral_sharpness_angle, bilateral_iter_number, relative_alpha, relative_offset, reconstruction_output_path_2);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("reconstruction failed: %s", e.what());
        return false;
    }

    ROS_INFO("reconstruction process completed. Output saved to %s", reconstruction_output_path_1.c_str());

    return true;
}
