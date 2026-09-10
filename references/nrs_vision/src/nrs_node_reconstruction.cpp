#include "nrs_io.h"
#include "nrs_math.h"
#include "nrs_mesh.h"
#include "nrs_callback.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nrs_reconstruction_node");
    ros::NodeHandle nh;

    // 각 경로는 상황에 맞게 수정
    nrs_callback n_callback;

    n_callback.reconstruction_input_path = "/home/nrs/catkin_ws/src/nrs_vision/pcd/registrated_pcd/workpiece.pcd";
    n_callback.reconstruction_yaml_path = "/home/nrs/catkin_ws/src/nrs_vision/config/params.yaml";
    n_callback.reconstruction_output_path_1 = "/home/nrs/catkin_ws/src/nrs_vision/mesh/workpiece.stl";
    n_callback.reconstruction_output_path_2 = "/home/nrs/catkin_ws/src/nrs_path/mesh/workpiece.stl";
    ros::ServiceServer rec_service = nh.advertiseService("/reconstruction", &nrs_callback::reconstructionServiceCallback, &n_callback);

    ROS_INFO("Reconstruction service is ready. Waiting for 'reconstruction' service calls...");
    ros::spin();
    return 0;
}