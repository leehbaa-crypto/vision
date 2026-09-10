#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <signal.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/robot/vpRobotUniversalRobots.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <sensor_msgs/JointState.h>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_UR_RTDE)

// Global variables for visual servoing and transformation
vpHomogeneousMatrix cMo, fMc;
bool cMo_validate = false;
ros::Time last_callback_time;
std::atomic<bool> control_thread_active(false);
vpHomogeneousMatrix cdMo(vpTranslationVector(0, 0, 0.45), vpRotationMatrix({1, 0, 0, 0, -1, 0, 0, 0, -1}));
// Robot instance
vpRobotUniversalRobots robot;

// Global pointers for publishers (used in publish thread)
ros::Publisher *g_fMc_pub = nullptr;
ros::Publisher *g_joint_states_pub = nullptr;

// Global mutex for protecting robot state access
std::mutex robot_mutex;
// Global variable to store fMc value from publishing loop in real-time
vpHomogeneousMatrix g_fMc_realtime;
std::mutex g_fMc_mutex;

std::vector<std::vector<double>> cubeVertices = {
    {1.1, 0.0, 0}, {0.45, 0.778, 0}, {0.45, 0.778, 0.7}, {1.1, 0.0, 0.7}, {0.398, -0.344, 0}, {0.184, 0.367, 0}, {0.184, 0.367, 0.7}, {0.398, -0.344, 0.7}};

std::vector<vpColVector> saved_joint_positions;
std::mutex saved_joint_mutex;

// Move the robot to a predefined initial position
void moveToInitialPosition(vpRobotUniversalRobots &robot)
{
  vpColVector q(6);
  q[0] = vpMath::rad(19.77);
  q[1] = vpMath::rad(-77.94);
  q[2] = vpMath::rad(-90.77);
  q[3] = vpMath::rad(-103.74);
  q[4] = vpMath::rad(89.39);
  q[5] = vpMath::rad(65.84);

  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
  robot.setPosition(vpRobot::JOINT_STATE, q);
  std::cout << "Moved to initial joint position." << std::endl;
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
}

// Check if the transformed point is within specified boundaries
bool checkOutOfBound(const vpTranslationVector &t, const vpHomogeneousMatrix &fMc,
                     const std::vector<std::vector<double>> &cubeVertices)
{
  // 1. 입력된 8개 점 확인 (직육면체의 꼭짓점)
  if (cubeVertices.size() != 8)
  {
    throw std::invalid_argument("cubeVertices must contain exactly 8 points.");
  }

  // 2. t와 fMc를 이용하여 점의 좌표를 구함 (동차 좌표 사용)
  vpColVector point(4);
  point[0] = t[0];
  point[1] = t[1];
  point[2] = t[2];
  point[3] = 1.0;

  vpMatrix fMc_mat = fMc;
  vpColVector transformed = fMc_mat * point; // 변환된 점

  // 변환된 점을 std::vector<double> (크기 3)로 변환
  std::vector<double> pt = {transformed[0], transformed[1], transformed[2]};
  // std::cout << "hand_X: " << pt[0] << "hand_Y: " << pt[1] << "hand_Z: " << pt[2] << std::endl;

  // 3. 직육면체 좌표계 정의 (cubeVertices는 아래 순서)
  //    0: front lower left
  //    1: front lower right
  //    2: front upper right
  //    3: front upper left
  //    4: back lower left
  //    5: back lower right
  //    6: back upper right
  //    7: back upper left
  //
  //    원점(origin): cubeVertices[0]
  const std::vector<double> &origin = cubeVertices[0];

  // 간단한 벡터 연산을 위한 lambda 함수들 (크기 3인 벡터라고 가정)
  auto subtract = [](const std::vector<double> &a, const std::vector<double> &b) -> std::vector<double>
  {
    std::vector<double> result(3);
    for (size_t i = 0; i < 3; ++i)
      result[i] = a[i] - b[i];
    return result;
  };

  auto dot = [](const std::vector<double> &a, const std::vector<double> &b) -> double
  {
    double sum = 0.0;
    for (size_t i = 0; i < 3; ++i)
      sum += a[i] * b[i];
    return sum;
  };

  // U = v1 - v0 (가로 방향), V = v3 - v0 (세로 방향), W = v4 - v0 (깊이 방향)
  std::vector<double> U = subtract(cubeVertices[1], origin);
  std::vector<double> V = subtract(cubeVertices[3], origin);
  std::vector<double> W = subtract(cubeVertices[4], origin);

  // 4. transformed 좌표와 원점 사이의 벡터(diff) 계산
  std::vector<double> diff = subtract(pt, origin);

  // 5. 각 축에 대해 diff의 성분을 내적(dot product)을 통해 구함
  double u = dot(diff, U) / dot(U, U);
  double v = dot(diff, V) / dot(V, V);
  double w = dot(diff, W) / dot(W, W);

  // 6. u, v, w 값이 모두 [0, 1]이면 내부에 있다고 판단 (경계 포함)
  bool inside = (u >= 0.0 && u <= 1.0 &&
                 v >= 0.0 && v <= 1.0 &&
                 w >= 0.0 && w <= 1.0);

  // 내부이면 false (즉, out-of-bound가 아님), 외부이면 true 반환
  return !inside;
}
// Callback to update transformation matrix from incoming message
void transformationMatrixCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  last_callback_time = ros::Time::now();
  if (msg->data.size() != 16)
  {
    cMo_validate = false;
    return;
  }

  for (auto val : msg->data)
  {
    if (std::isnan(val))
    {
      cMo_validate = false;
      return;
    }
  }

  // Obtain the real-time fMc value from publishLoop
  vpHomogeneousMatrix fMc_realtime_local;
  {
    std::lock_guard<std::mutex> lock(g_fMc_mutex);
    fMc_realtime_local = g_fMc_realtime;
  }

  vpTranslationVector t(msg->data[3], msg->data[7], msg->data[11]);
  // std::cout << fMc_realtime_local << std::endl;

  if (checkOutOfBound(t, fMc_realtime_local, cubeVertices))
  {
    ROS_INFO("out of bound!");
    cMo_validate = false;
    return;
  }

  vpRotationMatrix R;
  R[0][0] = msg->data[0];
  R[0][1] = msg->data[1];
  R[0][2] = msg->data[2];
  R[1][0] = msg->data[4];
  R[1][1] = msg->data[5];
  R[1][2] = msg->data[6];
  R[2][0] = msg->data[8];
  R[2][1] = msg->data[9];
  R[2][2] = msg->data[10];

  cMo.buildFrom(t, R);
  cMo_validate = true;
}

void callService(const std::string &service_name)
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>(service_name);
  std_srvs::Empty srv;
  if (client.call(srv))
  {
    ROS_INFO("[FSM] Visual servoing service called successfully: %s", service_name.c_str());
  }
  else
  {
    ROS_ERROR("[FSM] Failed to call %s", service_name.c_str());
  }
}

// Service callback to turn visual servoing on
bool handleVisualServoingOn(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
  control_thread_active.store(true);
  ROS_INFO("Visual servoing started.");
  return true;
}

// Service callback to turn visual servoing off
bool handleVisualServoingOff(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
  control_thread_active.store(false);
  moveToInitialPosition(robot);
  ROS_INFO("Visual servoing stopped.");
  return true;
}

bool handleScanMode(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
  // Set cdMo translation to (0.05, 0.05, 0.45) with same rotation
  cdMo = vpHomogeneousMatrix(
      vpTranslationVector(0.0, -0.03, 0.45),
      vpRotationMatrix({1, 0, 0,
                        0, -1, 0,
                        0, 0, -1}));

  return true;
}

bool handleFollowingMode(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
  // Reset cdMo translation back to (0, 0, 0.45) with same rotation
  cdMo = vpHomogeneousMatrix(
      vpTranslationVector(0, 0, 0.45),
      vpRotationMatrix({1, 0, 0,
                        0, -1, 0,
                        0, 0, -1}));
  return true;
}

void handleJointCommandCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  if (msg->data.size() != 6)
  {
    ROS_WARN("Received joint command size != 6. Ignored.");
    return;
  }

  vpColVector q(6);
  for (size_t i = 0; i < 6; ++i)
    q[i] = msg->data[i];

  {
    std::lock_guard<std::mutex> lock(robot_mutex);
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    robot.setPosition(vpRobot::JOINT_STATE, q);
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL); // 다시 velocity 제어로 복귀
  }

  ROS_INFO("Moved to commanded joint position from scan_joint_command.");
}

bool handleSaveViewPoint(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
  vpColVector q(6);
  {
    std::lock_guard<std::mutex> lock(robot_mutex);
    robot.getPosition(vpRobot::JOINT_STATE, q);
  }

  {
    std::lock_guard<std::mutex> lock(saved_joint_mutex);
    saved_joint_positions.push_back(q);
  }

  ROS_INFO("Saved current joint position to saved_joint_positions.");
  return true;
}

bool handleScanViewPoint(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
  std::lock_guard<std::mutex> lock(saved_joint_mutex);
  if (saved_joint_positions.empty())
  {
    ROS_WARN("No saved view points to scan.");
    return true;
  }

  for (auto it = saved_joint_positions.begin(); it != saved_joint_positions.end();)
  {
    {
      std::lock_guard<std::mutex> lock_robot(robot_mutex);
      robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
      robot.setPosition(vpRobot::JOINT_STATE, *it);
      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    }

    ROS_INFO("Moved to saved joint position.");
    callService("/scan");

    ros::Duration(3.0).sleep(); // 스캔 작업을 위한 대기 시간

    it = saved_joint_positions.erase(it); // 다녀온 위치는 제거
  }
  moveToInitialPosition(robot);

  ROS_INFO("Completed scan of all saved view points.");
  return true;
}

// High-frequency robot control loop running in its own thread
void controlLoop()
{
  ros::Rate control_rate(30); // e.g., 200Hz for control loop
  while (ros::ok())
  {
    if (control_thread_active.load())
    {
      // 제어 연산 수행
      vpColVector q(6);
      vpHomogeneousMatrix current_fMc;
      vpColVector v_c(6, 0.0);
      {
        std::lock_guard<std::mutex> lock(robot_mutex);
        current_fMc = robot.get_fMc();
        robot.getPosition(vpRobot::JOINT_STATE, q);

        // 최근 변환값이 있고 cMo_validate가 true이면 제어 연산 수행
        if ((ros::Time::now() - last_callback_time).toSec() <= 0.1 && cMo_validate)
        {

          vpHomogeneousMatrix cdMc = cdMo * cMo.inverse();

          vpServo task;
          task.setServo(vpServo::EYEINHAND_CAMERA);
          task.setInteractionMatrixType(vpServo::CURRENT);
          task.setLambda(vpAdaptiveGain(1.5, 0.8, 30));

          vpFeatureTranslation t(vpFeatureTranslation::cdMc), td(vpFeatureTranslation::cdMc);
          vpFeatureThetaU tu(vpFeatureThetaU::cdRc), tud(vpFeatureThetaU::cdRc);
          task.addFeature(t, td);
          task.addFeature(tu, tud);
          t.buildFrom(cdMc);
          tu.buildFrom(cdMc);

          v_c = task.computeControlLaw();
        }
        // 카메라 프레임 기준으로 속도 명령 설정
        robot.setVelocity(vpRobot::CAMERA_FRAME, v_c);
      }
    }
    control_rate.sleep();
  }
}

// Lower-frequency publishing loop for fMc and joint angles running in its own thread
void publishLoop()
{
  ros::Rate publish_rate(10); // e.g., 50Hz for publishing loop

  std::vector<std::string> joint_names = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"};

  while (ros::ok())
  {
    vpHomogeneousMatrix fMc_local;
    vpColVector q_local(6);

    {
      // Lock access to robot state
      std::lock_guard<std::mutex> lock(robot_mutex);
      fMc_local = robot.get_fMc();
      robot.getPosition(vpRobot::JOINT_STATE, q_local);
    }
    {
      // Update the global real-time fMc variable with the latest value from the robot
      std::lock_guard<std::mutex> lock(g_fMc_mutex);
      g_fMc_realtime = fMc_local;
    }
    // Prepare and publish fMc as a Float64MultiArray
    std_msgs::Float64MultiArray fMc_msg;
    fMc_msg.layout.dim.resize(2);
    fMc_msg.layout.dim[0].label = "rows";
    fMc_msg.layout.dim[0].size = 4;
    fMc_msg.layout.dim[0].stride = 16;
    fMc_msg.layout.dim[1].label = "cols";
    fMc_msg.layout.dim[1].size = 4;
    fMc_msg.layout.dim[1].stride = 4;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        fMc_msg.data.push_back(fMc_local[i][j]);
    if (g_fMc_pub)
      g_fMc_pub->publish(fMc_msg);

    // Prepare and publish the JointState message with joint angles
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();
    js.name = joint_names;
    js.position.resize(6);
    for (int i = 0; i < 6; ++i)
      js.position[i] = q_local[i];
    if (g_joint_states_pub)
      g_joint_states_pub->publish(js);

    publish_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nrs_node_visual_servoing");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("hand2cam_matrix", 10, transformationMatrixCallback);
  ros::Subscriber joint_command_sub = nh.subscribe("scan_joint_command", 10, handleJointCommandCallback);

  ros::Publisher fMc_pub = nh.advertise<std_msgs::Float64MultiArray>("fMc_matrix", 10);
  ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  g_fMc_pub = &fMc_pub;
  g_joint_states_pub = &joint_states_pub;

  ros::ServiceServer srv_on = nh.advertiseService("/visual_servoing_on", handleVisualServoingOn);
  ros::ServiceServer srv_off = nh.advertiseService("/visual_servoing_off", handleVisualServoingOff);
  ros::ServiceServer srv_scan_mode = nh.advertiseService("/scan_mode", handleScanMode);
  ros::ServiceServer srv_following_mode = nh.advertiseService("/following_mode", handleFollowingMode);
  ros::ServiceServer srv_save_view = nh.advertiseService("/save_view_point", handleSaveViewPoint);
  ros::ServiceServer srv_scan = nh.advertiseService("/scan_view_point", handleScanViewPoint);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Robot connection and initialization
  robot.connect("192.168.0.47");
  moveToInitialPosition(robot);
  vpPoseVector ePc;
  ePc.loadYAML("/home/nrs/catkin_ws/src/nrs_hand/ur_eMc.yaml", ePc);
  vpHomogeneousMatrix eMc(ePc);
  robot.set_eMc(eMc);

  // Start separate threads for high-frequency control and lower-frequency publishing
  std::thread control_thread(controlLoop);
  std::thread publish_thread(publishLoop);

  // Main thread waits for shutdown
  ros::waitForShutdown();

  control_thread.join();
  publish_thread.join();

  robot.setRobotState(vpRobot::STATE_STOP);
  return 0;
}

#else
int main()
{
  std::cout << "Missing ViSP dependencies." << std::endl;
  return 0;
}
#endif
