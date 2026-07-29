# 남은 통합 테스트 항목 (다음 라운드용)

## Context

전체 스택(Gazebo+로컬라이제이션+앱 브릿지) 통합 테스트를 두 라운드에 걸쳐
진행했다. 1라운드에서 연결/지도/수동조작/모드전이/move/진단STOP/GPS-오돔
일관성을 검증했고, 2라운드(추가 테스트)에서 뎁스카메라 장애물→지도 반영
파이프라인을 검증하다가 실제 버그 2개를 찾아 고쳤다. 아래는 2라운드 계획
중 아직 못 끝낸 항목들 — 다음에 이어서 진행할 것.

## 이번에 찾아서 고친 버그 (참고용 기록)

1. **`nav2_params.yaml`**: global/local costmap의 `depth_cloud` 관측 소스에
   `min_obstacle_height`가 없어서 뎁스카메라가 보는 바닥면(지면)까지 전부
   장애물로 잡혀 지도 전체가 하나의 거대한 점유 덩어리로 이어지던 문제.
   `min_obstacle_height: 0.05` 추가로 해결.
2. **`app_websocket_bridge.py`의 `_extract_obstacle_contours()`**: `cv2.RETR_EXTERNAL`이
   "테두리(링) 모양 도형의 구멍 안에 별개로 떨어져 있는 장애물"을 놓치는
   OpenCV의 알려진 함정에 걸려있었음(실측: `hierarchy[i][3]==-1`로 확인하면
   테두리와 박스 둘 다 최상위 외곽선인데도 RETR_EXTERNAL은 박스를 누락시킴).
   `RETR_CCOMP` + 부모 없는 컨투어만 필터링하는 방식으로 교체해 해결.
3. (1라운드) `mission_diagnostics_monitor.py`의 `DiagnosticStatus` 타입 버그 2개,
   `mission_state_machine.py`의 `in_error` 영구 래치(진단 회복/앱 명령으로
   조용히 안 풀리게) 설계 추가.

## 남은 테스트 항목

### 1. Collision monitor(FootprintApproach) 안전 정지
- `nav2_params.yaml`의 `collision_monitor`가 `/front_depth_camera/points`를
  유일한 관측 소스로 써서 충돌 1.2초 전 감속/정지시키는 로직
  (`FootprintApproach` 폴리곤) — 아직 실측 안 됨.
- **테스트 방법**: 장애물을 로봇 앞에 스폰(`gz service`로 primitive box —
  이번 라운드에서 쓴 방식/좌표계 그대로 재사용 가능) → KEY/CAL 모드에서
  move 명령으로 장애물을 향해 이동 → `/collision_monitor_state`가 반응하는지,
  `cmd_vel_nav_out`이 실제로 감속/정지하는지 확인.
- **중요한 구조적 사실**: 이 감시는 `cmd_vel_smoothed`(=nav2 경로, move
  명령/RUN 자율주행)만 지나간다. **조이스틱 수동조작(`cmd_vel_manual`)은
  collision_monitor를 아예 안 거치고 바로 `cmd_vel_arbiter`로 감** — 사람이
  조이스틱으로 몰 때는 자동 장애물 회피가 없다는 뜻. 의도한 설계인지 확인
  차 짚어볼 가치 있음(버그는 아님).

### 2. 진단 그룹 커버리지 공백
- `diagnostic_analyzers.yaml`엔 `hmi`(app_websocket_bridge)/`gps`
  (gps_covariance_filler) 그룹만 있고, `cmd_vel_twist_lpf.py`의
  diagnostic_updater("cmd_vel_out reception")는 어느 그룹에도 안 잡힘.
  `cmd_vel_arbiter`가 죽어서 `cmd_vel_out`이 끊겨도 진단→자동STOP 경로에
  안 걸림.
- **테스트 방법**: (a) 앱 연결 끊고 `/diagnostics_agg`에서 HMI 그룹 WARN
  뜨는지 확인(HMI 그룹 자체가 실제로 집계되는 걸 아직 실측 안 함),
  (b) `cmd_vel_arbiter` 죽여서 cmd_vel_twist_lpf 진단이 그룹 미분류로
  빠지는 걸 확인 → 그룹 추가할지 결정.

### 3. 주행 중 앱 연결 끊김에 대한 이중 안전장치
- KEY 모드로 실제 주행 중에 앱 연결을 끊었을 때, 독립된 두 안전장치
  (`key_manual_driver`의 1초 `control_state_stale_timeout_sec` vs
  `mission_state_machine`의 5초 명령 타임아웃)가 순서대로/올바르게 작동해서
  로봇이 확실히 멈추는지 실측 안 함(가만히 있을 때의 하트비트/재연결만
  테스트했음).

### 4. map_data 신뢰성 재전송(ack) 정상 경로
- 지금까진 "앱이 없어서 3번 재시도 후 포기"만 로그로 확인. 앱이 실제로
  `app_ack`를 보내서 재시도가 즉시 멈추는 정상 케이스, ack가 지연되는
  경우 재시도가 실제로 도는지는 확인 안 함.

## 테스트 시 유의사항 (이번 라운드에서 배운 것)

- **로봇을 `gz service .../set_pose`로 텔레포트하면 EKF/로컬라이제이션에
  안 알려진다** — GPS가 새 위치를 즉시 보고하면서 잠깐 위치가 어긋나고,
  그 사이 costmap에 잘못된 위치의 장애물 자국이 누적돼 안 지워질 수 있다
  (실제로 겪음: 대각선으로 늘어진 이상한 장애물 모양). 텔레포트 후에는
  nav2(costmap 호스트)를 재시작해서 누적 기록을 지우고 재확인할 것.
- **노드를 여러 개 짧은 시간에 연달아 재시작하면 EKF가 불안정해질 수
  있다** — 이번 라운드에서 `ekf_local`이 206초간 멈췄다가 이상한 값으로
  튀는 현상을 겪음(원인 미특정, 완전 재시작으로 해결). 개별 노드 재시작은
  최소화하고, 의심스러우면 전체 스택을 깨끗하게 재기동할 것.
- `ros2 topic echo`/`ros2 topic hz`가 일부 토픽(`/odom` 등)에서 간헐적으로
  아무 것도 못 받아오는 경우가 있음(CLI 자체의 QoS negotiation 타이밍
  이슈로 추정, 실제 토픽은 정상). rclpy로 직접 짧은 스크립트를 짜서
  확인하는 게 더 신뢰도 높음.
