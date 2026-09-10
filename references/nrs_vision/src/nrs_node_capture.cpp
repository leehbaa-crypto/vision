#include "nrs_callback.h"
#include <ros/ros.h>

#include <boost/filesystem.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nrs_node_capture");
    ros::NodeHandle nh;

    // nrs_callback 객체 생성 (생성자에서 필요한 초기화 수행)
    nrs_callback callback_obj;
    std::string package_path = ros::package::getPath("nrs_vision");

    // temp_pcd 디렉터리 비우기
    namespace fs = boost::filesystem;
    fs::path temp_dir(package_path + "/pcd/temp_pcd");
    if (fs::exists(temp_dir) && fs::is_directory(temp_dir))
    {
        for (auto &entry : fs::directory_iterator(temp_dir))
        {
            try
            {
                fs::remove_all(entry.path());
            }
            catch (const fs::filesystem_error &e)
            {
                ROS_WARN("Failed to remove %s: %s",
                         entry.path().c_str(), e.what());
            }
        }
        ROS_INFO("Cleared directory: %s", temp_dir.c_str());
    }
    else
    {
        ROS_WARN("Directory not found or not a directory: %s", temp_dir.c_str());
    }

    fs::path fMc_dir(package_path + "/pcd/fMc");
    if (fs::exists(fMc_dir) && fs::is_directory(fMc_dir))
    {
        for (auto &entry : fs::directory_iterator(fMc_dir))
        {
            try
            {
                fs::remove_all(entry.path()); // remove each file or subdirectory
            }
            catch (const fs::filesystem_error &e)
            {
                ROS_WARN("Failed to remove %s: %s",
                         entry.path().c_str(), e.what());
            }
        }
        ROS_INFO("Cleared directory: %s", fMc_dir.c_str());
    }
    else
    {
        ROS_WARN("Directory not found or not a directory: %s", fMc_dir.c_str());
    }

    callback_obj.n_io.pcd_file_path = package_path + "/pcd/temp_pcd/scan_cloud_";
    callback_obj.n_io.fMc_file_path = package_path + "/pcd/fMc/scan_fMc_";

    // "scan" 서비스 서버 등록: 호출되면 callback_obj.scanServiceCallback() 실행
    ros::ServiceServer scan_service = nh.advertiseService("/scan", &nrs_callback::scanServiceCallback, &callback_obj);

    ROS_INFO("Pointcloud capture node ready. Waiting for 'scan' service calls...");

    ros::spin();
    return 0;
}
