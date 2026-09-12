# Windows 이전·재개 — vision

작성: 2026-09-12. Ubuntu 원본과 코드 저장소를 보존한 뒤 실행 환경 기록·추가 자료를 비공개 보충 백업으로 준비했다. Windows에서의 실제 복원/장치 실행은 아직 검증하지 않았다.

문서 이동: [작업 규칙](../AGENTS.md) · [프로젝트 구조](PROJECT_CONTEXT.md) · [결과와 증거](FINAL_RESULTS.md) · [작업 이력](WORK_LOG.md) · [기존 복원 안내](../RESTORE_WINDOWS.md).

## 이 저장소의 대응

- 코드 저장소: https://github.com/leehbaa-crypto/vision
- Ubuntu 원본: `~/2026-1_urp`.
- GitHub에는 코드/자산의 보존 사본과 최신 인계 문서가 있다. 인계 문서를 별도로 수동 전송할 필요 없이 최신 main을 받는다.
- 전체 백업의 폴더 이름은 원본 이름이다. 새 clone에 백업을 통째로 덮어쓰지 말고 별도 폴더에 해제한 뒤 필요한 자료를 대응시킨다.

## 두 비공개 백업

1. [전체 원본 — 2026-09-10](https://github.com/leehbaa-crypto/ur10e/releases/tag/research-backup-2026-09-10): 모델·데이터·원본 작업 파일·Git 이력 bundle. 15개 조각과 PARTS_SHA256/restore_backup/README_RESTORE를 함께 받는다.
2. [Windows 이전 보충 — 2026-09-12](https://github.com/leehbaa-crypto/ur10e/releases/tag/research-migration-2026-09-12): 환경 정의·패키지/모듈 위치·설정·추가 코드/발표자료/진단 자료와 인계 문서 사본. README_MIGRATION/SUPPLEMENT_VERIFIED/verify_migration을 함께 받는다.

비공개 ur10e에 접근 가능한 GitHub 계정이 필요하다. 환경 원문, 추가 데이터와 개인 경로/네트워크 기록은 공개 저장소에 올리지 않고 보충 release에 보존한다.

## Ubuntu에서 확인한 사항

네 원본 폴더의 11,946개 파일/링크 항목이 기존 백업 manifest와 모두 일치했다. 변경·신규·누락 항목은 없다. 기존 GitHub release의 15개 조각/복원 도구의 크기·digest도 다시 확인했다. 별도 재다운로드의 최종 검사 기록은 보충 release의 DOWNLOAD_VERIFIED.json에서 확인한다.

시스템 Python 3.10.12, Conda base 3.12.7, Conda hamer 3.10.19, venv_mujoco_ros 3.10.12 환경을 기록했다. base에서 현재 ROS rclpy를 import하지 못했다. hamer에서는 rclpy/numpy/mujoco/mediapipe/pyrealsense2/trimesh/cv2/ikpy import가 성공했다. 일부 패키지는 사용자 site-packages에서 로드된다. 실제 모듈 경로와 초기 sandbox 제한으로 인한 실패/재검사 결과도 보존했다.

현재 `hamer` 환경에서 비전·ROS·MuJoCo 관련 주요 8개 모듈 import가 성공했다. 일부 모듈은 Conda 밖의 사용자 site-packages에서 로드되므로 환경 YAML만으로 복원이 끝나지 않는다.

현재 외부 RealSense 장치가 USB 목록에 없으므로 카메라 스트리밍·실제 로봇·전체 시뮬레이션 구동은 확인하지 않았다. 설치 환경 전체/OS 이미지를 옮기는 작업이 아니라 복원에 필요한 기록과 연구 파일을 보존한 작업이다.

## Windows에서의 재개 순서

1. 이 저장소의 최신 main을 clone/pull하고 AGENTS → PROJECT_CONTEXT → FINAL_RESULTS → WORK_LOG를 읽는다.
2. 두 비공개 백업을 다운로드한다. 분할/합친 원본·보충 압축·해제 결과를 함께 보관할 여유 공간으로 65GB 이상을 확보한다.
3. 원본 폴더에서 `py restore_backup.py`로 15개 조각의 해시를 검사하고 결합한다. 보충 폴더에서는 `py verify_migration.py --archive`로 전체 압축과 내부 파일을 검사한다.
4. 원본은 Linux/WSL의 별도 폴더에 해제하여 심볼릭 링크를 보존한다. 보충 자료도 별도 폴더에 해제한다.
5. 보충 verifier의 `--extracted`로 해제된 보충 자료를, Linux/WSL에서 `--original-root`로 해제된 네 원본 폴더를 검사한다.
6. `environment/`에서 패키지·모듈 위치와 환경 정의를 읽고 실제 실행 인터프리터를 재구성한다. YAML/pip/apt 목록은 검증된 일괄 설치 스크립트가 아니므로 Linux 패키지·ROS·로컬 editable 경로를 따로 확인한다.
7. central v17의 `central/visual` 경로와 launcher의 옛 홈 경로를 먼저 확인한다. 비전 v7/central v17/bridge v5 또는 v6의 조합은 실행 성공이 검증되지 않았다.
8. 문서/코드 분석은 먼저 재개할 수 있다. 실제 실행은 모델·토픽·좌표계·입력/출력 경로·장치 연결을 검증한 뒤 새 실험 폴더에서 수행한다.

`outside_research/`의 과거 코드·로그·화면녹화와 스크린샷은 보존/진단 자료다. 최종 결과나 현재 실행 코드로 자동 채택하지 않는다. 기존 백업과 동일해 생략된 파일은 `reports/OUTSIDE_FILES.json`의 matching_original_paths에서 위치를 찾는다.

## Ubuntu 원본을 유지해야 하는 종료 조건

Windows에서의 계정 접근·다운로드·압축 해제 확인 또는 외장 저장장치 사본 확보가 끝나기 전까지 Ubuntu 원본을 유지한다. 이 작업 시점에는 외장 디스크가 연결돼 있지 않다. Ubuntu에서 만든 보충 파일과 download_test 사본은 원본과 같은 물리 디스크이므로 포맷 대비 독립 디스크 사본이 아니다. 이번 준비 작업에서 파티션/OS/원본을 삭제하거나 변경하지 않았다.

