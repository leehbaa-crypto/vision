#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

#include <string>
#include <cstdlib>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>

#include <map>
#include <utility>
#include <sstream>
#include <iomanip>

#include <nrs_solution/HandGesture.h>

class CentralFSM
{
public:
  CentralFSM()
  {
    state = "standby";
    last_left = "";
    last_right = "";
    current_left = "";
    current_right = "";
    active_pid = -1;

    ros::NodeHandle nh;

    // 상태 전이 정의 (왼손, 오른손) -> 모드(또는 동작 설명)
    state_transitions["standby"] = {
        {{"paper", "paper"}, "scan_mode"},
        {{"paper", "pointing"}, "discrete_teach_mode"},
        {{"paper", "scissors"}, "continuous_teach_mode"},
        {{"paper", "rock"}, "follow_mode"},
        {{"paper", "okay"}, "path_planning_mode"}};

    state_transitions["scan"] = {
        {{"", "rock"}, "terminate scan"},
        {{"", "pointing"}, "scan"},
        {{"", "scissors"}, "registration & reconstruction"}};

    state_transitions["discrete_teach"] = {
        {{"", "rock"}, "terminate discrete_teach"},
        {{"", "pointing"}, "save discrete_teaching waypoints"}};

    state_transitions["continuous_teach"] = {
        {{"", "rock"}, "terminate continuous_teach"},
        {{"", "pointing"}, "save continuous_teaching waypoints"}};

    state_transitions["follow"] = {
        {{"", "rock"}, "terminate follow"}};

    state_transitions["path_planning"] = {
        {{"", "rock"}, "terminate path_planning"},
        {{"", "pointing"}, "generate path using discrete_waypoints"},
        {{"", "scissors"}, "generate path using continuous waypoints"},
        {{"okay", "okay"}, "execute planned path"}};

    state_timer = nh.createTimer(
        ros::Duration(1.0),
        &CentralFSM::stateTimerCallback,
        this);

    gesture_sub = nh.subscribe("/hand_gesture", 10, &CentralFSM::gestureCallback, this);

    last_gesture_time = ros::Time::now();
    gestureHoldThreshold = ros::Duration(3.0); // 3초 이상 유지되어야 동작
    printStateBannerMono(state);
    ROS_INFO("[FSM] Central FSM node initialized. Current state: standby");
  }

private:
  // -------------------- 출력 유틸 (고정폭 ASCII 대시보드) --------------------
  static std::string fit(const std::string &s, size_t w)
  {
    // w 너비 맞추기: 넘치면 마지막에 '…' 붙이고, 부족하면 공백 패딩
    if (s.size() <= w)
      return s + std::string(w - s.size(), ' ');
    if (w == 0)
      return "";
    if (w == 1)
      return "…";
    return s.substr(0, w - 1) + "…";
  }

  static void clearAndHome()
  {
    // 전체 지우기 + 커서 홈으로 이동 (터미널에서만 효과, 로그 리다이렉트시 그냥 문자열)
    std::cout << "\033[2J\033[H";
  }

  static std::string renderDashboardMono(
      const std::string &state,
      const std::string &left,
      const std::string &right,
      double held_sec,
      const std::map<std::pair<std::string, std::string>, std::string> &mapping)
  {
    const int W = 80; // 전체 폭(원하면 조절)
    std::ostringstream oss;
    std::string hr(W, '-');

    oss << "+" << hr << "+\n";
    oss << "| " << fit("FSM DASHBOARD", W - 2) << "|\n";
    oss << "+" << hr << "+\n";

    std::ostringstream row1;
    row1 << "State: " << state;
    oss << "| " << fit(row1.str(), W - 2) << "|\n";

    std::ostringstream row2;
    row2 << "Left: " << (left.empty() ? "-" : left)
         << "   Right: " << (right.empty() ? "-" : right)
         << "   Held: " << std::fixed << std::setprecision(1) << held_sec << "s";
    oss << "| " << fit(row2.str(), W - 2) << "|\n";

    oss << "+" << hr << "+\n";
    oss << "| " << fit("Next transitions (L,R) -> mode", W - 2) << "|\n";
    oss << "+" << hr << "+\n";

    if (mapping.empty())
    {
      oss << "| " << fit("(no mappings)", W - 2) << "|\n";
    }
    else
    {
      int idx = 1;
      for (const auto &kv : mapping)
      {
        const auto &g = kv.first;
        const auto &m = kv.second;
        std::string gl = g.first.empty() ? "-" : g.first;
        std::string gr = g.second.empty() ? "-" : g.second;

        std::ostringstream line;
        line << std::setw(2) << idx++ << ") "
             << "(" << fit(gl, 12) << ", " << fit(gr, 12) << ") -> "
             << m;
        oss << "| " << fit(line.str(), W - 2) << "|\n";
      }
    }
    oss << "+" << hr << "+";
    return oss.str();
  }

  static void printStateBannerMono(const std::string &s)
  {
    clearAndHome();
    std::ostringstream oss;
    std::string hr(80, '=');
    oss << hr << "\n"
        << "  FSM STATE -> " << s << "\n"
        << hr << "\n";
    std::cout << oss.str() << std::flush;
  }

  // ------------------------------ ROS 구성 요소 ------------------------------
  ros::Subscriber gesture_sub;
  ros::Timer state_timer;
  std::string state;
  pid_t active_pid;
  std::string node_to_kill;

  // 제스처 상태
  std::string current_left, current_right;

  // 제스처 지속 확인용 변수
  std::string last_left, last_right;
  ros::Time last_gesture_time;
  ros::Duration gestureHoldThreshold;

  // 상태 전이 테이블: state -> ( (L,R) -> 설명/모드 )
  std::map<std::string,
           std::map<std::pair<std::string, std::string>, std::string>>
      state_transitions;

  // ------------------------------ 타이머 콜백 ------------------------------
  void stateTimerCallback(const ros::TimerEvent &)
  {
    // 현재 상태의 전이 목록
    std::map<std::pair<std::string, std::string>, std::string> mapping;
    auto it = state_transitions.find(state);
    if (it != state_transitions.end())
      mapping = it->second;

    // 제스처 홀드 시간
    double held = (ros::Time::now() - last_gesture_time).toSec();

    // 같은 위치에 덮어쓰기
    clearAndHome();
    std::string dash = renderDashboardMono(state, current_left, current_right, held, mapping);
    std::cout << dash << std::endl
              << std::flush;

    // 요약 로그(정렬 깨짐 방지용, 5초에 한 번)
    ROS_INFO_THROTTLE(5.0, "[FSM] state=%s L=%s R=%s held=%.1fs",
                      state.c_str(), current_left.c_str(), current_right.c_str(), held);
  }

  // ------------------------------ 제스처 콜백 ------------------------------
  void gestureCallback(const std_msgs::String::ConstPtr &msg)
  {
    // "Left:xxx,Right:yyy" 파싱
    std::string data = msg->data;
    std::string left, right;
    size_t pL = data.find("Left:");
    size_t pC = data.find(",Right:");
    if (pL != std::string::npos && pC != std::string::npos)
    {
      left = data.substr(pL + 5, pC - (pL + 5));
      right = data.substr(pC + 7);
    }

    current_left = left;
    current_right = right;

    // 지속 확인 (3초 이상 유지 필요)
    if (!checkPersistentGesture(left, right))
      return;

    /* ------------------------ Standby Mode ------------------------ */
    if (state == "standby")
    {
      if (left == "paper" && right == "paper")
      {
        ROS_INFO("[FSM] Gesture: scan mode trigger");
        callService("/scan_mode");
        callService("/visual_servoing_on");
        launch("nrs_solution", "nrs_scan.launch", "scan");
      }
      else if (left == "paper" && right == "pointing")
      {
        ROS_INFO("[FSM] Gesture: discrete teaching trigger");
        callService("/scan_mode");
        callService("/visual_servoing_on");
        launch("nrs_solution", "nrs_discrete_teach.launch", "discrete_teach");
      }
      else if (left == "paper" && right == "scissors")
      {
        ROS_INFO("[FSM] Gesture: continuous teaching trigger");
        callService("/scan_mode");
        callService("/visual_servoing_on");
        launch("nrs_solution", "nrs_continuous_teach.launch", "continuous_teach");
      }
      else if (left == "paper" && right == "rock")
      {
        ROS_INFO("[FSM] Gesture: follow mode trigger");
        callService("/following_mode");
        callService("/visual_servoing_on");
        launch("nrs_solution", "nrs_follow.launch", "follow");
      }
      else if (left == "paper" && right == "okay")
      {
        ROS_INFO("[FSM] Gesture: path planning mode trigger");
        callService("/visual_servoing_off");
        killNode("nrs_node_visual_servoing");
        launch("nrs_path", "path_planning.launch", "path_planning");
      }
    }

    /* ------------------------ Scan Mode ------------------------ */
    else if (state == "scan")
    {
      if (left == "" && right == "rock")
      {
        ROS_WARN("[FSM] Gesture: terminating scan mode");
        callService("/visual_servoing_off");
        terminateMode();
      }
      else if (left == "" && right == "pointing")
      {
        callService("/save_view_point");
        last_left = "";
        last_right = "";
      }
      else if (left == "" && right == "scissors")
      {
        callService("/visual_servoing_off");
        callService("/scan_view_point");
        callService("/registration");
        callService("/reconstruction");
        terminateMode();
      }
    }

    /* ------------------------ Continuous Teaching Mode ------------------------ */
    else if (state == "continuous_teach")
    {
      if (left == "" && right == "rock")
      {
        ROS_WARN("[FSM] Gesture: ending continuous teaching mode");
        callService("/continuous_teaching_end");
        callService("/visual_servoing_off");
        callService("/interpolate_continuous_waypoints");
        terminateMode();
      }
      else if (left == "" && right == "pointing")
      {
        callService("/continuous_teaching_save");
      }
    }

    /* ------------------------ Discrete Teaching Mode ------------------------ */
    else if (state == "discrete_teach")
    {
      if (left == "" && right == "rock")
      {
        ROS_WARN("[FSM] Gesture: ending discrete teaching mode");
        callService("/discrete_teaching_end");
        callService("/visual_servoing_off");
        terminateMode();
      }
      else if (left == "" && right == "pointing")
      {
        callService("/discrete_teaching_save");
        last_left = "";
        last_right = "";
      }
    }

    /* ------------------------ Follow Mode ------------------------ */
    else if (state == "follow")
    {
      if (left == "" && right == "rock")
      {
        ROS_WARN("[FSM] Gesture: ending follow mode");
        callService("/visual_servoing_off");
        terminateMode();
      }
    }

    /* ------------------------ Path Planning Mode ------------------------ */
    else if (state == "path_planning")
    {
      if (left == "" && right == "rock")
      {
        ROS_WARN("[FSM] Gesture: ending path_planning mode");
        terminateMode();
      }
      else if (left == "" && right == "pointing")
      {
        callService("/straight");
        callService("/interpolate");
        callService("/simulation");
      }
      else if (left == "" && right == "scissors")
      {
        callService("/continuous");
        callService("/interpolate");
        callService("/simulation");
      }
      else if (left == "okay" && right == "okay")
      {
        // launch("rtde_handarm", "nrs_ur10.launch", "path_planning");
        callService("/Iteration_set");
        callService("/Playback_execution");
        callService("/Playback_execution");
      }
    }
  }

  // ------------------------------ 유틸 함수들 ------------------------------
  bool checkPersistentGesture(const std::string &l, const std::string &r)
  {
    ros::Time now = ros::Time::now();
    if (l != last_left || r != last_right)
    {
      last_left = l;
      last_right = r;
      last_gesture_time = now;
      return false;
    }
    return (now - last_gesture_time >= gestureHoldThreshold);
  }

  bool launch(const std::string &pkg, const std::string &file, const std::string &mode)
  {
    if (state != "standby")
      return false;

    state = mode;
    printStateBannerMono(state);
    ROS_INFO("[FSM] Launching %s mode...", mode.c_str());

    pid_t pid = fork();
    if (pid == 0)
    {
      execlp("roslaunch", "roslaunch", pkg.c_str(), file.c_str(), (char *)nullptr);
      _exit(EXIT_FAILURE);
    }
    active_pid = pid;
    return true;
  }

  void killNode(const std::string &node_name)
  {
    std::string cmd = "rosnode kill " + node_name;
    int ret = system(cmd.c_str());
    if (ret != 0)
      ROS_ERROR("Failed to kill node %s (return code %d)", node_name.c_str(), ret);
    else
      ROS_INFO("Killed node %s", node_name.c_str());
  }

  void killLaunch(const std::string &pkg, const std::string &file)
  {
    std::string pattern = "roslaunch " + pkg + " " + file;
    std::string cmd = "pkill -f \"" + pattern + "\"";
    int ret = system(cmd.c_str());
    if (ret != 0)
      ROS_ERROR("Failed to kill launch [%s %s] (ret=%d)", pkg.c_str(), file.c_str(), ret);
    else
      ROS_INFO("Killed launch [%s %s]", pkg.c_str(), file.c_str());
  }

  void terminateMode()
  {
    if (active_pid > 0)
    {
      kill(active_pid, SIGINT);
      active_pid = -1;
      state = "standby";
      printStateBannerMono(state);
      ROS_INFO("[FSM] Returned to standby mode");
    }
  }

  void publishCommand(const std::string &cmd_str)
  {
    std_srvs::Empty srv;
    ros::ServiceClient client = ros::NodeHandle().serviceClient<std_srvs::Empty>("/nrs_command/" + cmd_str);
    if (client.call(srv))
      ROS_INFO("[FSM] Command triggered via service: %s", cmd_str.c_str());
    else
      ROS_ERROR("[FSM] Failed to call command service for: %s", cmd_str.c_str());
  }

  void callService(const std::string &service_name)
  {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>(service_name);
    std_srvs::Empty srv;
    if (client.call(srv))
      ROS_INFO("[FSM] Service called: %s", service_name.c_str());
    else
      ROS_ERROR("[FSM] Failed to call %s", service_name.c_str());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nrs_node_central_FSM");
  CentralFSM fsm;
  ros::spin();
  return 0;
}
