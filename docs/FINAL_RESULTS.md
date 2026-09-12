# FINAL_RESULTS — vision

작성 기준: 2026-09-10, 최종 문서 검토: 2026-09-12. 저장소 `vision`, 기존 HEAD `750956c0dc1aba1ed233df90618ebc8147f19fbe`. 정적 파일 분석과 보존 기록에 근거하며 새 장치 실험은 수행하지 않았다.

문서 이동: [작업 규칙](../AGENTS.md) · [프로젝트 구조](PROJECT_CONTEXT.md) · [결과와 증거](FINAL_RESULTS.md) · [작업 이력](WORK_LOG.md).

## 최종적으로 인정되는 실험

**UNKNOWN.** 사용자는 연구 방향을 양손 로봇 비전 제어로 설명했지만 최종 실험 ID·실행 버전·조건·평가 로그를 지정하지 않았다. 현재 확인한 루트/central/mp_hand_tracking/simulation_mujoco 경로에서 최종 성능 표를 뒷받침하는 실험 기록을 찾지 못했다. references와 외부 라이브러리의 자료를 현재 연구 실험으로 간주하지 않는다.

## 확정된 성능 수치

**확정된 실험 성능 수치 없음.** 제스처 정확도, 3D 추종 RMSE, 지연, 실측 FPS, 접촉력 성능, 두 로봇 팔 동기 오차는 UNKNOWN이다.

다음은 코드 설정/구조로만 확인됐다.

| 항목 | 코드에서 확인한 값 | 해석 한계 |
|---|---|---|
| 센서 요청 | 640×480 color/depth, 30fps | 실제 처리 FPS 아님 |
| 비전 루프 gate | 0.05초 | 목표 제한이며 처리 지연 미측정 |
| 중앙 제어 timer | 0.05초 | 실제 주기 보장 아님 |
| 상태 송신 timer | 0.5초 | 통신 지연 측정값 아님 |
| 입력 timeout | 1.5초 | ESTOP 전이 조건이며 안전 인증 아님 |
| 중앙 IK | 최대 15회, error norm break 0.001 | 실제 TCP 정확도/수렴 보장 아님 |
| 입력/출력 구성 | 양손 제스처, 관절 명령 6개 | 두 로봇 팔 구동 근거 아님 |

## 사용하지 말아야 할 중간/외부 결과

- `central/debug1.py`, DEBUG_VERTICAL 모드, `oldversion/`, bridge 버전별 출력은 최종 실험 채택 근거가 없다.
- `hamer`/ViTPose의 학습·벤치마크·inference speed 문서는 외부 모델 자료이며 이 카메라·제어 시스템의 결과가 아니다.
- `references/nrs_hand/.../*.csv`에는 학습/수집 데이터가 있다. 파일 존재만으로 현재 제스처 정확도를 산출하거나 임의로 train/test를 섞지 않는다.
- ROS 1 인수인계의 standby/teaching 동작을 ROS 2 v17에서 검증했다고 쓰지 않는다.
- v5 대비 v6의 높은 gain이나 force offset/EMA 추가를 실제 진동 감소·힘 추종 성능 향상 수치로 바꾸어 쓰지 않는다.

## 해석과 trade-off

FOLLOWING은 3D 깊이 입력, GUIDED_FOLLOWING은 정규화 pad와 mesh ray intersection을 사용하므로 오차의 원인·평가 좌표계가 다르다. 필터링, 다회 제스처 확인, stale timeout은 안정적 입력과 응답 속도 사이의 설계 선택이나 실측 절충량은 UNKNOWN이다. 중앙 v17은 `/ft_sensor`를 구독하지 않는다. 고정 음수 TCP offset을 쓰는 접촉을 폐루프 힘 제어의 검증으로 제시하지 않는다.

## 최종 확정 TODO

먼저 PROJECT_CONTEXT에 기록한 central mesh 경로/launcher 불일치를 해결 대상으로 확인한다. 기준 bridge/central/vision 조합을 고정하고, 좌표계·양손 의미·카메라 보정·실제 로봇 개수를 문서화한다. 동일 조건의 입력 및 관절/force 로그, GT, 실패 사례와 반복 수를 보존한 뒤 평가한다. 정확한 입력·코드·산출물 대응과 연구자의 최종 채택이 있어야 최종 수치를 확정할 수 있다.
