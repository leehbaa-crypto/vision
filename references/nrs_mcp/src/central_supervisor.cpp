#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <errno.h>

#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>

struct LaunchSpec {
  std::string pkg;
  std::string file;
};

class CentralSupervisor {
public:
  CentralSupervisor(ros::NodeHandle& nh): nh_(nh), running_(true) {
    // 관리할 론치 정의
    specs_["scan"]  = {"nrs_mcp", "multiview_scan.launch"};
    specs_["teach"] = {"nrs_solution", "nrs_fsm.launch"};
    specs_["path"]  = {"nrs_path", "path_planning.launch"};
    specs_["control"]  = {"nrs_mcp", "robot_control.launch"};

    // 서비스 등록
    srv_scan_start_  = nh_.advertiseService("/scan_package_start",  &CentralSupervisor::handleScanStart,  this);
    srv_scan_end_    = nh_.advertiseService("/scan_package_end",    &CentralSupervisor::handleScanEnd,    this);
    srv_teach_start_ = nh_.advertiseService("/teach_package_start", &CentralSupervisor::handleTeachStart, this);
    srv_teach_end_   = nh_.advertiseService("/teach_package_end",   &CentralSupervisor::handleTeachEnd,   this);
    srv_path_start_ = nh_.advertiseService("/path_planning_package_start", &CentralSupervisor::handlePathPlanningStart, this);
    srv_path_end_   = nh_.advertiseService("/path_planning_package_end",   &CentralSupervisor::handlePathPlanningEnd,   this);
    srv_control_start_ = nh_.advertiseService("/robot_control_package_start", &CentralSupervisor::handleRobotControlStart, this);
    srv_control_end_   = nh_.advertiseService("/robot_control_package_end",   &CentralSupervisor::handleRobotControlEnd,   this);

    // 자식 프로세스 리퍼 스레드
    reaper_ = std::thread(&CentralSupervisor::reaperLoop, this);

    ROS_INFO("[central_supervisor] ready.");
  }

  ~CentralSupervisor() {
    running_ = false;
    if (reaper_.joinable()) reaper_.join();

    // 남은 론치들 안전 종료
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto& kv : pids_) {
      safeTerminate(kv.second);
    }
    pids_.clear();
  }

private:
  // ===== 서비스 핸들러 =====
  bool handleScanStart(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    return start("scan");
  }
  bool handleScanEnd(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    return stop("scan");
  }
  bool handleTeachStart(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    return start("teach");
  }
  bool handleTeachEnd(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    return stop("teach");
  }
  bool handlePathPlanningStart(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    return start("path");
  }
  bool handlePathPlanningEnd(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    return stop("path");
  }
  bool handleRobotControlStart(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    return start("control");
  }
  bool handleRobotControlEnd(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    return stop("control");
  }

  // ===== 실행 로직 =====
  bool start(const std::string& key) {
    auto it = specs_.find(key);
    if (it == specs_.end()) {
      ROS_ERROR("unknown key: %s", key.c_str());
      return false;
    }

    std::lock_guard<std::mutex> lk(mtx_);

    // 이미 실행 중이면 OK 처리하고 패스
    if (pids_.count(key) && processAlive(pids_[key])) {
      ROS_WARN("[%s] already running with pid %d", key.c_str(), pids_[key]);
      return true;
    }

    const auto& spec = it->second;
    pid_t pid = fork();
    if (pid < 0) {
      ROS_ERROR("[%s] fork failed: %s", key.c_str(), strerror(errno));
      return false;
    }

    if (pid == 0) {
      // child: roslaunch 실행
      // 표준 입출력 분리하고 세션 리더로 만들어 부모 신호 영향 최소화
      setsid();
      execlp("roslaunch", "roslaunch", spec.pkg.c_str(), spec.file.c_str(), (char*)nullptr);
      // execlp 실패 시
      _exit(EXIT_FAILURE);
    }

    // parent
    pids_[key] = pid;
    ROS_INFO("[%s] launched: roslaunch %s %s (pid=%d)", key.c_str(), spec.pkg.c_str(), spec.file.c_str(), pid);
    return true;
  }

  bool stop(const std::string& key) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!pids_.count(key)) {
      ROS_WARN("[%s] no running process", key.c_str());
      return true;
    }
    pid_t pid = pids_[key];
    bool ok = safeTerminate(pid);
    if (ok) {
      pids_.erase(key);
      ROS_INFO("[%s] terminated", key.c_str());
    } else {
      ROS_ERROR("[%s] terminate failed for pid %d", key.c_str(), pid);
    }
    return ok;
  }

  // ===== 유틸 =====
  bool processAlive(pid_t pid) {
    if (pid <= 0) return false;
    if (kill(pid, 0) == 0) return true;
    return false;
  }

  bool safeTerminate(pid_t pid) {
    if (pid <= 0) return true;

    // 1) SIGINT
    kill(pid, SIGINT);
    if (waitForExit(pid, 3000)) return true;

    // 2) SIGTERM
    kill(pid, SIGTERM);
    if (waitForExit(pid, 2000)) return true;

    // 3) SIGKILL
    kill(pid, SIGKILL);
    if (waitForExit(pid, 1000)) return true;

    return false;
  }

  bool waitForExit(pid_t pid, int timeout_ms) {
    const int step = 50;
    int waited = 0;
    int status = 0;
    while (waited < timeout_ms) {
      pid_t r = waitpid(pid, &status, WNOHANG);
      if (r == pid) return true;
      if (r == -1 && errno == ECHILD) return true; // 이미 수거됨
      std::this_thread::sleep_for(std::chrono::milliseconds(step));
      waited += step;
    }
    return false;
  }

  void reaperLoop() {
    // 좀비 수거 스레드
    while (running_) {
      int status = 0;
      pid_t r = waitpid(-1, &status, WNOHANG);
      if (r > 0) {
        std::lock_guard<std::mutex> lk(mtx_);
        // pid → key 역탐색해서 정리
        for (auto it = pids_.begin(); it != pids_.end(); ) {
          if (it->second == r) {
            ROS_INFO("[reaper] child pid %d exited for key %s", r, it->first.c_str());
            it = pids_.erase(it);
          } else {
            ++it;
          }
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceServer srv_scan_start_;
  ros::ServiceServer srv_scan_end_;
  ros::ServiceServer srv_teach_start_;
  ros::ServiceServer srv_teach_end_;
  ros::ServiceServer srv_path_start_;
  ros::ServiceServer srv_path_end_;
  ros::ServiceServer srv_control_start_;
  ros::ServiceServer srv_control_end_;

  std::map<std::string, LaunchSpec> specs_;
  std::map<std::string, pid_t> pids_;
  std::mutex mtx_;

  std::thread reaper_;
  std::atomic<bool> running_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "central_supervisor");
  ros::NodeHandle nh;

  // Ctrl+C 시그널 들어오면 ROS가 정리하고 소멸자에서 자식 종료
  CentralSupervisor sup(nh);
  ros::spin();
  return 0;
}
