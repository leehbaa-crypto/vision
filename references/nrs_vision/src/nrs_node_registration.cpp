#include "nrs_io.h"
#include "nrs_math.h"
#include "nrs_pointcloud.h"
#include "nrs_callback.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nrs_node_registration");
    ros::NodeHandle nh;

    // 각 경로는 상황에 맞게 수정

    nrs_pointcloud n_pcl;
    nrs_callback n_callback;

    n_callback.registration_input_pcd_path = "/home/nrs/catkin_ws/src/nrs_vision/pcd/temp_pcd";
    n_callback.registration_input_fMc_path = "/home/nrs/catkin_ws/src/nrs_vision/pcd/fMc";
    n_callback.registration_yaml_path = "/home/nrs/catkin_ws/src/nrs_vision/config/params.yaml";
    n_callback.registration_output_path = "/home/nrs/catkin_ws/src/nrs_vision/pcd/registrated_pcd/workpiece.pcd";
    // remove all files in registration_input_pcd_path

    ros::ServiceServer reg_service = nh.advertiseService("/registration", &nrs_callback::registrationServiceCallback, &n_callback);

    ROS_INFO("Registration service is ready. Waiting for 'registration' service calls...");
    ros::spin();
    return 0;
}