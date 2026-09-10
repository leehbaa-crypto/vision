# Windows에서 연구 자료 확인·복원

보존일: 2026-09-10. 원본 개발 환경은 Ubuntu 22.04 / ROS 2 Humble이며, 코드는 Linux 경로와 장치 연결에 의존할 수 있습니다. Windows에서 파일 열람은 가능하지만 Windows 네이티브 실행을 검증하지 않았습니다.

1. Git을 설치하고 해당 저장소를 clone하거나 GitHub의 Download ZIP으로 받습니다.
2. 코드, 인수인계 문서, 실험 결과를 확인합니다. 비전 모델 가중치 등 일반 Git에서 제외한 파일은 원본 백업에서 복원해야 합니다.
3. ROS 2 코드를 계속 개발할 때는 기존 Ubuntu 환경을 별도로 복구하세요. WSL을 쓰더라도 USB 카메라와 실제 로봇 네트워크 설정은 추가 확인이 필요합니다.
4. `/home/leehyunbin/`, `/home/chomi14/` 같은 절대 경로를 새 환경에 맞게 수정하고 의존성을 설치합니다. 기존 build/install 파일을 그대로 실행하기보다 소스에서 다시 빌드하세요.

저장소 대응:
- senior_design ← ~/senior_design
- vision ← ~/2026-1_urp (구 UR10e-vision-control)
- ur10e ← ~/2025_winter_urp + ~/ros2_ws

`ENVIRONMENT_20260831.txt`는 이전에 수집한 환경 참고 기록이며, 재설치용 lockfile은 아닙니다.
