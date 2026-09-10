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

## 전체 원본 다운로드

모델·데이터를 포함한 세 연구의 원본은 [비공개 전체 백업](https://github.com/leehbaa-crypto/ur10e/releases/tag/research-backup-2026-09-10)에서 받습니다. `leehbaa-crypto` 계정으로 로그인해야 합니다.

릴리스의 모든 `.001`~마지막 조각과 `PARTS_SHA256.json`, `restore_backup.py`, `README_RESTORE.md`를 같은 폴더에 내려받은 뒤 `py restore_backup.py`(Windows) 또는 `python3 restore_backup.py`를 실행하세요. 조각별 SHA-256을 확인하고 하나의 tar.gz로 합칩니다. 복구 스크립트는 Python 표준 라이브러리만 사용합니다.

약 15.9GB의 분할 압축본, 같은 크기의 합친 압축본, 약 18.6GB의 압축 해제 결과를 함께 보관하려면 여유 공간을 55GB 이상 확보하세요. Linux 심볼릭 링크는 Linux/WSL에서 압축을 풀어 복원합니다. 원본 파일 해시와 Git 이력 bundle은 압축 안의 `backup_metadata/`에 있습니다.

개인 홈 전체와 운영체제·설치된 Anaconda 환경 자체는 백업 대상이 아닙니다. 원본 연구 폴더의 Git 설정·인증 정보 및 Python 캐시는 제외했습니다.
