# PROJECT_CONTEXT — vision

작성 기준: 2026-09-10, 최종 문서 검토: 2026-09-12. 저장소 `vision`, 기존 HEAD `750956c0dc1aba1ed233df90618ebc8147f19fbe`. 정적 파일 분석과 보존 기록에 근거하며 새 장치 실험은 수행하지 않았다.

문서 이동: [작업 규칙](../AGENTS.md) · [프로젝트 구조](PROJECT_CONTEXT.md) · [결과와 증거](FINAL_RESULTS.md) · [작업 이력](WORK_LOG.md).

## 연구 목적과 범위

사용자는 UR10e 기반 연구가 막바지에 “양손 로봇 비전 제어”로 바뀌었다고 설명했고 저장소 이름을 `vision`으로 정리했다. 실제로 확인한 경로는 **양손 제스처/좌표를 입력받아 6관절 UR10e 모델 하나를 제어하는 ROS 2/MuJoCo 파이프라인**이다. 독립된 두 로봇 팔의 동기 제어가 구현·실험됐는지는 UNKNOWN이다. “양손 입력”을 “두 로봇 팔 제어 완료”로 확대 해석하지 않는다.

## 주요 파일과 역할

| 경로 | 실제 역할 / 상태 |
|---|---|
| `mp_hand_tracking/realsense_mediapipe_v7.py` | RealSense 깊이 + MediaPipe 양손 제스처, 3D target/2D guided target 생성; 분석 기준 후보 |
| `central/nrs_central_controller_v17.py` | FSM, 좌표 변환, 표면 ray intersection, FK/Jacobian/numerical IK, 관절 명령; class 이름은 `CentralControllerV21` |
| `simulation_mujoco/ros2_mujoco_pure_v5.py` | URDF/mesh를 결합한 MuJoCo bridge, 표면 선택·관절/힘 송신; README가 현재 지목하는 후보 |
| `simulation_mujoco/ros2_mujoco_pure_v6.py` | 다른 gain, 힘 영점/필터 구현; class 이름은 V9. 최종인지 UNKNOWN |
| `simulation_mujoco/nrs_gesture_launcher.py` | `LAUNCH:`를 받으면 hardcoded 홈 경로의 unversioned bridge 실행; 위 v5와 같다고 가정 금지 |
| `simulation_mujoco/ur10e_combined.urdf` | v5/v6가 실제 읽는 기구 입력 |
| `simulation_mujoco/ur10e_update.xml`, `temp_robot.xml`, `visual/`, `collision/`, 표면 STL | 다른 모델/시뮬레이션 자산. 파일 존재와 실행 시 로드 경로를 구분 |
| `central/oldversion/`, `mp_hand_tracking/oldversion/`, `central/debug1.py` | 과거 및 debug; 최종 결과로 자동 채택 금지 |
| `hamer/`, `hamer/third-party/ViTPose/` | HaMeR 대안 경로/외부 코드. v7 light 파이프라인에서 import하지 않음 |
| `mano_v1_2/`, `references/` | MANO 도구, 과거 NRS 연구·학습 데이터·코드. 원본 모델은 비공개 백업 참고 |
| `인수인계.pdf`, `인수인계_text.txt` | 이전 ROS 1/catkin/roslaunch 경로가 섞인 인수인계. 현재 ROS 2 실행법과 구분 |

## ROS 2 인터페이스

| 토픽 | 타입 | 방향 |
|---|---|---|
| `/hand_gesture` | `std_msgs/String` | 비전 → 중앙; `Left:...,Right:...` |
| `/target_pose` | `geometry_msgs/Point` | 비전 → 중앙; 카메라 3D 좌표 |
| `/guided_target` | `geometry_msgs/Point` | 비전 → 중앙; pad 정규화 좌표, z=0 |
| `/surface_command` | `std_msgs/String` | 비전 → 중앙/bridge/(선택한 경우 launcher); `LAUNCH:파일명` |
| `/joint_commands` | `std_msgs/Float64MultiArray` | 중앙 → bridge; 관절 6개 |
| `/joint_states` | `sensor_msgs/JointState` | bridge → 중앙, sensor-data QoS |
| `/robot_status` | `std_msgs/String` | 중앙 → 비전 |
| `/ft_sensor` | `geometry_msgs/WrenchStamped` | bridge → 관측자; 읽은 중앙 v17에는 힘 피드백 구독 없음 |

이 인터페이스는 `ur10e/ros2_ws`의 `/isaac_joint_commands`, `/ftsensor/measured_Cvalue`와 다르다. 이름뿐 아니라 메시지 타입도 다르므로 코드들을 바로 혼합하지 않는다.

## 실행 구조와 상태 전이

- v7는 640×480 color/depth, 30fps 스트림을 요청하고 MediaPipe에는 320×240 영상을 보낸다. 루프에 0.05초 gating이 있다. 실측 처리속도는 UNKNOWN이다.
- 손 분류 Left/Right를 코드 안에서 교환하고 display를 flip한다. 사용자 좌/우와 카메라 좌/우 의미를 새 환경에서 확인해야 한다.
- NOT READY에서 표면 선택 후 양손 `okay`로 `LAUNCH:`를 송신한다. 중앙이 표면 로드에 성공해야 IDLE이 된다.
- 중앙의 주요 상태는 NOT READY, IDLE, ESTOP, RESETTING, FOLLOWING, GUIDED_FOLLOWING, DEBUG_VERTICAL이다. 제스처 누적 7회 조건과 상태별 전이 제한이 있다. README의 STANDBY/TEACHING/PLAYBACK 목록만으로 현재 FSM을 설명하지 않는다.
- FOLLOWING: 3D 좌표 변환/범위 제한/필터링 → 위치·방향 IK. GUIDED_FOLLOWING: 2D pad 좌표 → 표면 ray intersection → normal/높이에 맞춘 IK.
- 중앙 timer 0.05초, 상태 송신 0.5초, 입력 timeout 1.5초, IK 최대 15회 반복, 위치 step 제한 0.02는 코드 값이다. 실제 안정성·속도·정확도 측정값이 아니다.
- v17의 `tcp_z_offset=-0.01`은 표면 안쪽으로 목표를 설정하는 값이다. 이 경로의 힘 제어 성능을 검증한 것은 아니다.
- v5/v6는 각각 단일 6관절 PD/물리 bridge 후보이다. v6는 v5와 gain 및 힘 보정 방식이 다르므로 숫자가 높다는 이유로 교체하지 않는다.

## 새 머신 실행 순서 — 먼저 아래 경로 문제 해결 필요

1. ROS 2/rclpy/RealSense/MediaPipe/MuJoCo/trimesh 환경과 센서 연결을 확인한다. 루트 `requirements.txt`는 현재 없다. 기존 설치 안내의 해당 명령은 준비 완료 상태가 아니다.
2. v5 또는 v6 중 기준 후보 하나를 선정하고, 중앙 v17 및 비전 v7의 토픽 계약을 확인한다. 아직 성공한 실행 조합은 UNKNOWN이다.
3. 중앙·bridge를 먼저 준비한 뒤 비전의 surface 선택/launch 흐름을 확인한다. launcher를 함께 사용하는 경우 unversioned bridge가 추가 실행될 수 있으므로 중복 bridge를 피한다.
4. 모델 로드 성공, NOT READY→IDLE, 실제 joint 상태 수신을 확인한 후 FOLLOWING/GUIDED_FOLLOWING/ESTOP를 검증한다.

## 확인된 문제와 해결 상태

- **미해결, 정적 확인:** 중앙 v17은 `central/visual/<surface>`를 찾지만 해당 디렉터리가 없다. 표면 STL은 `simulation_mujoco/`에 있다. 예외를 `except Exception: pass`로 숨겨 NOT READY에 남을 가능성이 있다. 이번에는 코드 수정하지 않았다.
- **미해결:** launcher는 `~/2026-1_urp/simulation_mujoco/ros2_mujoco_pure.py`를 실행한다. 새 clone 이름 `vision`과 README의 v5 지목이 모두 다르다.
- **미해결:** v5/v6의 URDF 텍스트에 `<body>` 정규식으로 ft site 삽입을 시도하는 부분이 있다. 실제 MuJoCo 센서/site 생성이 성공하는지 실행 검증 필요. 코드의 `sys.exit(1)`을 성공으로 해석하지 않는다.
- **미해결:** SETUP_EXTERNAL_ASSETS는 `vitpose_base`를 적지만 보존된 대형 가중치는 `vitpose+_huge/wholebody.pth`이다. 모델 경로와 사용 버전을 백업에서 확인한다.
- **보존 해결:** 기존 GitHub main은 README 위주였고 실제 연구는 master에 있었다. 두 이력을 보존하고 main/master에 연구 파일을 올렸다.
- **보존 해결:** HaMeR와 5개 reference 모듈이 gitlink만 있던 상태에서 실제 소스 파일을 포함하도록 보존했다. 무시되던 시뮬레이션 메시도 추가했다. 모델 가중치·전체 데이터는 비공개 백업에 있다.
- **UNKNOWN:** 최종 제어 조합, 두 로봇 팔 구현, 실기 성능, 제스처 정확도/추종 오차/지연, ROS 1 reference와 현 ROS 2 코드의 실험 대응.

## 다른 머신에서 공통으로 알아야 할 사항

- 저장소 대응: `senior_design` ← `~/senior_design`; `vision` ← `~/2026-1_urp`; `ur10e` ← `~/2025_winter_urp` + `~/ros2_ws`.
- [전체 원본 백업](https://github.com/leehbaa-crypto/ur10e/releases/tag/research-backup-2026-09-10)은 **비공개 ur10e 저장소**에 있다. 접근 권한이 있는 GitHub 계정이 필요하다.
- 백업: 15,936,308,499 bytes, 15개 분할 압축 파일. 원본 11,946개 파일/링크 항목을 대조했고, Git 이력 bundle 13개를 보존했다. 조각별 GitHub SHA-256 일치를 확인했다. 이 숫자는 백업 검증 수치이며 연구 성능이 아니다.
- 모든 분할 파일, `PARTS_SHA256.json`, `restore_backup.py`, `README_RESTORE.md`를 함께 받는다. `ARCHIVE_VERIFIED.json`에 검증 결과가 있다. 복원 스크립트의 정상 결합/손상 거부를 작은 샘플로 검사했다.
- 압축 안에는 원래 폴더 이름이 유지된다. 저장소 루트에 무조건 덮어쓰지 말고 별도 폴더에 해제한 뒤 대응시킨다. GitHub 사본에는 원본 폴더 이후에 추가한 안내·복구 소스가 있으므로 함께 보존한다.
- `.git`, `.git_backup`, Python 캐시는 제외했다. 원본 manifest에 기록된 `2026-1_urp/hamer/third-party/ViTPose/.git_backup` 포인터 1개도 Git 메타데이터 제외 대상이다. 개인 홈 전체·OS·설치된 Anaconda 환경 자체를 백업한 것은 아니다.
- `ENVIRONMENT_20260831.txt`는 이전에 수집된 참고 환경이며 lockfile이 아니다. Ubuntu 22.04/ROS 2 Humble 계열 기록과 Anaconda Python 3.12.7 출력이 섞여 있다. 각 실제 실행 인터프리터·환경의 정확한 대응은 UNKNOWN이다.
- Windows 네이티브 실행과 WSL USB/로봇 연결은 검증하지 않았다. Linux 절대 경로·심볼릭 링크·ROS 환경을 먼저 확인한다. 실제 장치 없이 하드웨어 시험을 통과했다고 기록하지 않는다.
