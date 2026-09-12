# WORK_LOG — vision

작성 기준: 2026-09-10, 최종 문서 검토: 2026-09-12. 저장소 `vision`, 기존 HEAD `750956c0dc1aba1ed233df90618ebc8147f19fbe`. 정적 파일 분석과 보존 기록에 근거하며 새 장치 실험은 수행하지 않았다.

문서 이동: [작업 규칙](../AGENTS.md) · [프로젝트 구조](PROJECT_CONTEXT.md) · [결과와 증거](FINAL_RESULTS.md) · [작업 이력](WORK_LOG.md).

## 대화에서 확정된 의도

사용자는 Windows로 OS/컴퓨터를 옮기기 전에 세 연구를 GitHub에 올리고 `senior_design`, `ur10e`, `vision`으로 구분하기를 원했다. 대용량 원본 업로드도 명시적으로 승인했다. 현재 요청은 실제 파일을 바탕으로 AGENTS와 docs 3개를 **새로 생성하는 문서 작업**이다. 결과가 없으면 추측하지 않고 UNKNOWN/TODO로 남긴다.

현재 작업 위치는 `/home/leehyunbin`이며 단일 연구 Git 루트가 아니었다. 앞서 업로드한 사본 `/home/leehyunbin/research_upload_20260910/vision`에서 이 문서를 작성했다. 원래 연구 폴더와 이 사본은 별개이므로 새 머신에서는 이 저장소와 연결된 버전을 기준으로 삼는다.

## 이전 Codex 보존·업로드 작업 (2026-09-10)

기존 `UR10e-vision-control`을 `vision`으로 변경했다. main의 초기 README 이력과 master의 연구 이력을 보존했다. gitlink로만 기록된 HaMeR 및 NRS reference 5개 모듈을 실제 파일로 포함하고 무시되던 시뮬레이션 mesh를 추가했다. 원본 로컬 `2026-1_urp`의 origin도 새 이름을 가리키게 했다. 이전 보존 작업에서 README 제목/실행 안내를 수정했지만 실행 성공을 검증한 것은 아니다.

공통으로 Git 브랜치/로컬 변경을 확인하고, 원본과 중복 ZIP을 비교하고, 코드·문서·자산을 업로드했다. 새 `RESTORE_WINDOWS.md`와 환경 참고 파일을 추가했다. 원격 main 커밋과 로컬 HEAD 일치를 확인했다. 알고리즘을 개선하거나 로봇 실험을 새로 수행한 작업은 없었다.

홈의 `robotics.zip`은 다른 연구 ZIP을 담는 중복 묶음이었다. `2025_winter_urp.zip`, `2026-1_urp.zip`의 연구 파일 목록/크기 대조에서 현재 폴더에 없는 파일은 확인되지 않았다. `2026-1_urp_hb.zip`은 ZIP 헤더가 손상됐으며 읽을 수 있던 1,618개 연구 파일 목록에 현재 폴더에 없는 파일은 없었다. 손상 ZIP 자체를 완전 복원했다고 하지 않는다.

## 이번 handoff 작성에서 확인한 내용

중앙 v17의 잘못된 surface 경로와 숨겨진 예외, launcher의 예전 홈 경로, 버전/class 이름 불일치를 기록했다. 양손 입력과 단일 6관절 모델을 확인했다. 최종 두 로봇 제어·정확도·힘 제어 성능은 증거가 없어 UNKNOWN으로 두었다. 실제 하드웨어·MuJoCo 모델 로드를 수행하지 않았다.

핵심 파일을 읽고 필요한 구간을 검색했다. 아래 목록은 확인한 주요 증거이며 저장소의 모든 외부 라이브러리와 모든 과거 버전을 전수 코드 리뷰했다는 뜻은 아니다.

- `README.md`
- `SETUP_EXTERNAL_ASSETS.md`
- `RESTORE_WINDOWS.md`
- `인수인계_text.txt`
- `central/nrs_central_controller_v17.py`
- `mp_hand_tracking/realsense_mediapipe_v7.py`
- `simulation_mujoco/ros2_mujoco_pure_v5.py`
- `simulation_mujoco/ros2_mujoco_pure_v6.py`
- `simulation_mujoco/nrs_gesture_launcher.py`
- `simulation_mujoco/ur10e_update.xml`

## 중요한 판단

- 코드 설정, 파일에 적힌 주장, 재계산 일치, 최종 채택 실험을 서로 구분했다.
- 현재 README와 코드의 불일치는 handoff에 적고 기존 파일은 수정하지 않았다.
- 버전 번호나 파일명의 '최종'을 신뢰해 최종 기준선을 임의 선택하지 않았다.
- 사용자 원본 데이터·기존 요약·그림·코드·README를 그대로 보존한다. 앞으로 고칠 문제는 PROJECT_CONTEXT의 TODO로 남긴다.

## 재개 순서 / 아직 해야 할 작업

1. AGENTS → PROJECT_CONTEXT → FINAL_RESULTS를 읽고 현재 HEAD/작업 트리 상태를 확인한다.
2. 비공개 원본 백업을 별도 폴더에 복구하고, 실행에 필요한 파일만 정확한 상대 경로에 대응시킨다.
3. PROJECT_CONTEXT의 미해결 경로/환경/실행 조합 문제를 먼저 확인한다. 이번 문서 생성은 해당 문제를 고친 작업이 아니다.
4. 연구자에게 최종 실험 ID와 실행 코드/조건을 확인하거나 보관 자료에서 증거를 찾는다. 확인할 때까지 UNKNOWN을 유지한다.
5. 로봇/시뮬레이션 재실행 전 기준선 입력과 원본 해시, 새 출력 디렉터리를 확보한다.
6. 검증한 항목만 결과 문서에 추가하고, 미실행 항목을 통과 처리하지 않는다.

## 이번 문서 작업의 검증 범위

2026-09-10 기록: 세 저장소의 기존 파일 총 3,439개 SHA-256이 문서 작성 전후 모두 같았고, 각 저장소에 요청된 새 문서 4개만 추가됐다. 기존 tracked 파일 diff는 비어 있었다.

2026-09-12 재개·완료: 남아 있던 12개 문서를 읽고 핵심 코드·설정·기존 결과 요약과 다시 대조했다. 각 문서에 상대 링크와 최종 검토일을 추가하고, 확인되지 않은 최종 실험/실행 조합은 UNKNOWN/TODO로 유지했다. 문서 작성 작업은 완료됐으며 위 재개 순서는 이후 연구 작업을 위한 것이다.

이번 검증은 Git 메타데이터와 요청 문서 12개를 제외한 모든 기존 파일을 대상으로 한다. ignore된 파일도 포함하며 저장소별 파일 수는 senior_design 50개, ur10e 1,208개, vision 2,181개로 총 3,439개이다. 정렬된 상대 경로와 파일 내용(심볼릭 링크는 대상 경로)의 SHA-256 집계값이 재개 전후 같았다. HEAD와 tracked 파일도 그대로였고, 미추적 파일은 저장소마다 요청 문서 4개뿐이다. 12개 문서의 코드 펜스와 로컬 Markdown 링크 대상도 확인했다. 로봇·카메라·MuJoCo 실행이나 ROS 빌드 검증은 수행하지 않았다.

여기까지는 AGENTS.md와 docs 3개를 신규 산출물로 남긴 문서 작업이었다. 코드·raw data·기존 요약·그림·README는 수정하지 않았다. 이 시점에는 문서가 로컬 미추적 상태였다. 2026-09-10에 게시한 원본 백업 release에는 이후 작성한 인계 문서가 들어 있지 않다.

## Windows 이전 준비 — 2026-09-12 후속 요청

사용자가 이전 준비 전체를 진행하도록 요청하여 인계 문서의 GitHub 게시, 실제 실행 환경 기록, 백업 재점검, 연구 폴더 밖 자료 보존으로 작업 범위를 확장했다. docs/MIGRATION_WINDOWS.md를 추가하고 각 저장소의 문서 5개를 커밋·게시한다. 게시 커밋/원격 파일 내용 확인 결과는 비공개 보충 release의 PUBLISHED_HANDOFF.json에 기록한다.

현재 원본 11,946개 항목을 기존 manifest와 대조해 변경·신규·누락이 없음을 확인했다. 기존 release의 게시 상태와 15개 조각/복원 도구의 digest도 재확인했다. 기존 원본 백업은 갱신하지 않고 별도 보충 release를 준비한다. GitHub 재다운로드와 압축 내부 내용 검사의 최종 기록은 보충 release의 DOWNLOAD_VERIFIED.json을 확인한다.

환경 기록은 시스템 Python/Conda base/hamer/venv_mujoco_ros별 버전·패키지·모듈 위치·pip 목록, Conda YAML/from-history/explicit 목록, OS 패키지·ROS shell 설정·실행 이력 발췌·USB/네트워크·RealSense 빌드 설정을 포함한다. base의 Python 3.12와 현재 Python 3.10 ROS 확장은 import가 맞지 않는다. hamer에서는 주요 8개 패키지 import가 성공했지만 일부는 사용자 site-packages에서 로드된다. 초기 sandbox udev 제한과 제한 밖에서의 재검사 결과를 구분해 보존했다. 실제 카메라가 연결되지 않아 스트리밍/로봇 실행은 검증하지 않았다.

연구 폴더 밖 파일 4,971개를 조사하여 동일 내용 986개는 기존 백업 대응 경로를 기록하고 3,985개/1,492,088,563 bytes를 보충 자료에 복사했다. 제어 코드·입력·발표자료·SDK 소스/설정·추가 연구 ZIP·ROS 로그·화면녹화/스크린샷을 보존하며, 미분류 진단 자료를 최종 실험으로 지정하지 않는다. 로컬 RealSense Linux 런타임 파일 6개도 별도 참고용으로 보존했다. 기존 코드·실험 데이터를 변경하지 않았다.

비공개 [보충 백업](https://github.com/leehbaa-crypto/ur10e/releases/tag/research-migration-2026-09-12)의 README_MIGRATION.md와 SUPPLEMENT_VERIFIED.json에 범위/해시를 기록한다. Windows에서는 최신 저장소와 두 release를 받으면 되므로 인계 문서를 따로 수동 복사하지 않아도 된다. Windows에서 실제 복원하거나 외장 디스크로 복사하는 단계는 현 환경에서 수행하지 않았다. 연결된 외장 디스크가 없고 download_test도 같은 물리 디스크에 있으므로 독립 디스크 백업으로 간주하지 않는다. 해당 외부 확인이 끝나기 전까지 Ubuntu 원본을 유지한다.

## 다른 머신에서 공통으로 알아야 할 사항

- 저장소 대응: `senior_design` ← `~/senior_design`; `vision` ← `~/2026-1_urp`; `ur10e` ← `~/2025_winter_urp` + `~/ros2_ws`.
- [전체 원본 백업](https://github.com/leehbaa-crypto/ur10e/releases/tag/research-backup-2026-09-10)은 **비공개 ur10e 저장소**에 있다. 접근 권한이 있는 GitHub 계정이 필요하다.
- 백업: 15,936,308,499 bytes, 15개 분할 압축 파일. 원본 11,946개 파일/링크 항목을 대조했고, Git 이력 bundle 13개를 보존했다. 조각별 GitHub SHA-256 일치를 확인했다. 이 숫자는 백업 검증 수치이며 연구 성능이 아니다.
- 모든 분할 파일, `PARTS_SHA256.json`, `restore_backup.py`, `README_RESTORE.md`를 함께 받는다. `ARCHIVE_VERIFIED.json`에 검증 결과가 있다. 복원 스크립트의 정상 결합/손상 거부를 작은 샘플로 검사했다.
- 압축 안에는 원래 폴더 이름이 유지된다. 저장소 루트에 무조건 덮어쓰지 말고 별도 폴더에 해제한 뒤 대응시킨다. GitHub 사본에는 원본 폴더 이후에 추가한 안내·복구 소스가 있으므로 함께 보존한다.
- `.git`, `.git_backup`, Python 캐시는 제외했다. 원본 manifest에 기록된 `2026-1_urp/hamer/third-party/ViTPose/.git_backup` 포인터 1개도 Git 메타데이터 제외 대상이다. 개인 홈 전체·OS·설치된 Anaconda 환경 자체를 백업한 것은 아니다.
- `ENVIRONMENT_20260831.txt`는 이전에 수집된 참고 환경이며 lockfile이 아니다. Ubuntu 22.04/ROS 2 Humble 계열 기록과 Anaconda Python 3.12.7 출력이 섞여 있다. 각 실제 실행 인터프리터·환경의 정확한 대응은 UNKNOWN이다.
- Windows 네이티브 실행과 WSL USB/로봇 연결은 검증하지 않았다. Linux 절대 경로·심볼릭 링크·ROS 환경을 먼저 확인한다. 실제 장치 없이 하드웨어 시험을 통과했다고 기록하지 않는다.
