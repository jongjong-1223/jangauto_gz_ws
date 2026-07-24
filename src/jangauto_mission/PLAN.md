# jangauto_mission (YASMIN 상태머신) + diagnostics 파이프라인

## Context

`jangauto_mission` 패키지는 현재 완전히 빈 `ament_cmake` 스켈레톤이다 (`CMakeLists.txt`/`package.xml`만 존재, `rclpy` 의존성도 없음). 조사 결과 `/state_command`/`/robot_state`라는 상태 토픽은 이미 코드에 등장하지만, 그걸 쓰는 유일한 코드(`jangauto_hmi/scripts/reference/app_bridge.py`)는 **빌드되지 않는 참고용 파일**이라 실제로는 이 워크스페이스에 "현재 시스템 상태가 뭔지 아는 노드"가 전혀 없다. `app_wifi_tx.py`(역시 참고용)의 `REQUIRED_NODES`에는 `state_manager`/`state_machine_executor`라는 이름이 언급되지만 실제 구현은 없다 — 즉 `jangauto_mission`이 메꿔야 할 자리가 정확히 이거다.

실제로 지금 돌고 있는 새 프로토콜은 `app_websocket_bridge.py`가 앱에서 받은 JSON을 그대로 재발행하는 `/app/control_state` (String, `{"sw_bits":16,...}` 형태)이다. 사용자와 확인한 범위:
- FSM은 `/app/control_state`를 **직접 구독**해서 `sw_bits` 값으로 STOP/KEY/CAL/ALIGN/RUN 5개 상태를 전환하고 `/robot_state`(String)를 발행하는 것까지만 담당한다.
- `cmd_vel` 게이팅(예전 `app_bridge.py`가 하던, KEY 상태에서만 이동명령 허용하는 로직)은 **이번 범위 밖** — 별도 노드로 나중에.
- 상태머신 라이브러리는 **YASMIN** (`ros-jazzy-yasmin`, `-ros`, `-msgs`, `-viewer` — apt로 존재 확인됨, rosdep 키는 없어서 수동 설치 필요).

두 번째로, 전체 노드를 `/diagnostics`로 모니터링하는 파이프라인도 함께 구축한다. 조사 결과 diagnostics 관련 코드는 워크스페이스에 전혀 없다 (grep 0건). 실제로 돌고 있는 커스텀 파이썬 노드는 `app_websocket_bridge`와 `gps_covariance_filler` 단 둘뿐(그 외 `robot_state_publisher`/`ekf_local`/`ekf_global`/`navsat_transform`은 외부 패키지 노드라 자체 diagnostics가 없음). 사용자와 확인한 범위:
- 이번엔 **우리 커스텀 노드 2개**(`app_websocket_bridge`, `gps_covariance_filler`)에만 `diagnostic_updater`를 직접 붙인다. 외부 노드용 토픽 생존감시 워치독은 다음 단계로 미룬다.
- `diagnostic_aggregator`로 집계하고, 지금은 **`rqt_robot_monitor`**로만 확인한다. (나중에 `app_websocket_bridge`가 `/diagnostics_agg`를 구독해서 앱으로 미러링할 계획이 있다는 것만 염두에 두고, 그때 걸리적거리지 않게 이름/그룹 구조를 잡는다.)

## Part 1 — `jangauto_mission`: YASMIN 상태머신

### 패키지 전환
`jangauto_mission`은 지금 `rclpy`조차 의존하지 않는 빈 `ament_cmake` 패키지다. 워크스페이스의 기존 관례(`jangauto_hmi`, `jangauto_gps_driver`, `jangauto_application` 전부 `ament_cmake` + `install(PROGRAMS scripts/*.py ...)` 방식, `ament_python`이 아님)를 그대로 따른다.

- `jangauto_mission/CMakeLists.txt`: `install(PROGRAMS scripts/mission_state_machine.py DESTINATION lib/${PROJECT_NAME} ...)` + `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})` 추가 (jangauto_gps_driver/CMakeLists.txt 패턴 그대로 재사용).
- `jangauto_mission/package.xml`: `<exec_depend>rclpy</exec_depend>`, `<exec_depend>std_msgs</exec_depend>`, `<exec_depend>yasmin</exec_depend>`, `<exec_depend>yasmin_ros</exec_depend>`, `<exec_depend>yasmin_msgs</exec_depend>`, `<exec_depend>yasmin_viewer</exec_depend>` 추가. **주의**: `rosdep resolve yasmin` 실패 확인됨(rosdep 키 없음) — `sudo apt-get install ros-jazzy-yasmin ros-jazzy-yasmin-ros ros-jazzy-yasmin-msgs ros-jazzy-yasmin-viewer`를 사용자가 직접 실행해야 함 (이전 `python3-websockets`/`python3-zeroconf`와 동일 패턴).

### `jangauto_mission/scripts/mission_state_machine.py` (신규)
- `sw_bits` → 상태 이름 매핑은 예전 `app_bridge.py`의 `sw_command_map`과 동일하게: `16→STOP`, `8→KEY`, `4→CAL`, `2→ALIGN`, `1→RUN`.
- YASMIN의 `yasmin_ros.MonitorState`를 사용 — 토픽을 구독하다가 조건에 맞는 메시지가 오면 바로 outcome을 반환하는 State. 5개 상태(STOP/KEY/CAL/ALIGN/RUN) 각각을 `MonitorState(String, '/app/control_state', outcomes=["STOP","KEY","CAL","ALIGN","RUN"], monitor_handler=...)`로 만들고, `monitor_handler`에서 JSON 파싱 → `sw_bits` 추출 → 매핑된 상태명을 outcome으로 반환. 모든 상태가 서로 전이 가능한 완전연결 그래프로 구성 (지금 요구사항엔 STOP→RUN 같은 강제 경유 규칙이 없으므로 기존 동작과 동일하게 유지 — 나중에 규칙이 필요해지면 이 전이 테이블만 손보면 됨).
- 상태가 바뀔 때(= monitor_handler가 새 outcome을 반환하는 시점) `/robot_state` (String)를 발행. 같은 상태로의 재요청(예: 이미 KEY인데 KEY 메시지가 또 옴)은 재발행하지 않도록 이전 상태와 비교.
- `yasmin_viewer.YasminViewerPub`으로 FSM 구조/현재상태를 발행 (yasmin_viewer 웹 UI에서 시각적으로 볼 수 있음 — diagnostics와는 별개의 보너스 모니터링 채널).
- JSON 파싱 실패/`sw_bits` 키 없음 등은 무시하고 현재 상태 유지 (경고 로그만).

### `jangauto_mission/launch/mission.launch.py` (신규)
`mission_state_machine.py` 노드 하나를 실행하는 얇은 launch 파일 (다른 패키지들과 동일한 패턴).

### `jangauto_bringup` 연동
`tracked_v3.launch.py`에 `mission = IncludeLaunchDescription(...)` 추가 (다른 include들과 같은 자리에), `jangauto_bringup`의 `CMakeLists.txt`/`package.xml`에 `jangauto_mission` 의존성 추가.

## Part 2 — 커스텀 노드 2개에 diagnostic_updater 추가

### `jangauto_hmi/scripts/app_websocket_bridge.py`
`diagnostic_updater.Updater(self)` 생성, 하나의 진단 태스크 등록해서 매 주기(예: 1Hz, 기존 heartbeat 타이머와 별개 타이머 혹은 같이 묶어도 됨) 아래를 보고:
- WS 서버가 실제로 리슨 중인지 (`self._ws_server is not None`)
- mDNS 등록 상태 (`self._mdns_service_info is not None`)
- 기존에 이미 추적 중인 `self._ws_last_link_alive` — `True`면 OK, `False`면 WARN(연결된 앱 없음/끊김), 서버 자체가 안 떠 있으면 ERROR.
`jangauto_hmi/package.xml`에 `<exec_depend>diagnostic_updater</exec_depend>` 추가.

### `jangauto_drivers/jangauto_gps_driver/scripts/gps_covariance_filler.py`
같은 패턴으로 `diagnostic_updater.Updater` 추가 — `navsat` 콜백에서 마지막 수신 시각을 기록해두고, 진단 콜백에서 "최근 N초 이내 수신했는가"로 OK/WARN(데이터 끊김)/ERROR(한번도 수신 안 함) 판단. `jangauto_gps_driver/package.xml`에 `<exec_depend>diagnostic_updater</exec_depend>` 추가.

## Part 3 — `diagnostic_aggregator` 연동

### `jangauto_bringup/config/diagnostic_analyzers.yaml` (신규)
analyzers 설정으로 시작은 두 그룹만 정의 (나중에 노드가 늘어나면 그룹만 추가하면 됨):
```yaml
analyzers:
  ros__parameters:
    analyzers:
      hmi:
        type: diagnostic_aggregator/GenericAnalyzer
        path: HMI
        contains: ['app_websocket_bridge']
      gps:
        type: diagnostic_aggregator/GenericAnalyzer
        path: GPS
        contains: ['gps_covariance_filler']
```
(정확한 `contains`/`path` 매칭 문자열은 실제 diagnostic_updater가 내보내는 `DiagnosticStatus.name` 포맷을 빌드 후 `ros2 topic echo /diagnostics`로 한 번 확인하고 맞춰 조정 — analyzer 매칭은 이름 문자열 기반이라 실측이 필요함.)

### `jangauto_bringup/launch/diagnostics.launch.py` (신규)
`diagnostic_aggregator` 패키지의 `aggregator_node`를 위 yaml을 파라미터로 실행하는 얇은 launch 파일. `tracked_v3.launch.py`에 include 추가.

### 의존성
`jangauto_bringup/package.xml`/`CMakeLists.txt`에 `diagnostic_aggregator` 추가. `sudo apt-get install ros-jazzy-diagnostic-aggregator ros-jazzy-rqt-robot-monitor`도 사용자가 직접 설치 (rosdep 키 미확인 상태라 동일하게 수동 설치 필요할 가능성 높음, 빌드 시 확인).

## 실행 순서

1. `jangauto_mission` CMakeLists/package.xml 전환 + `mission_state_machine.py` + `mission.launch.py` 작성
2. `app_websocket_bridge.py`, `gps_covariance_filler.py`에 diagnostic_updater 추가
3. `jangauto_bringup/config/diagnostic_analyzers.yaml` + `diagnostics.launch.py` 작성
4. `tracked_v3.launch.py`에 mission + diagnostics include 추가, 관련 package.xml/CMakeLists 의존성 정리
5. 필요한 apt 패키지들 사용자가 설치 (yasmin 계열, diagnostic_aggregator, rqt_robot_monitor)
6. 빌드 후 실제 `/diagnostics` 메시지 이름 포맷 확인하고 analyzer yaml의 `contains` 값 보정

## 검증

- `ros2 launch jangauto_bringup tracked_v3.launch.py` 이후 `ros2 topic echo /robot_state` 띄워두고, 테스트 WebSocket 클라이언트로 `sw_bits`를 16→8→4→2→1 순서로 보내면서 STOP→KEY→CAL→ALIGN→RUN이 정확히 순서대로 찍히는지 확인 (이전 세션에서 쓰던 `ws_test_client.py` 패턴 재사용 가능).
- `ros2 topic echo /diagnostics`로 두 커스텀 노드의 DiagnosticStatus가 찍히는지, 앱 연결을 끊었을 때 `app_websocket_bridge`쪽이 WARN으로 바뀌는지 확인.
- `ros2 run rqt_robot_monitor rqt_robot_monitor` 띄워서 HMI/GPS 그룹이 트리에 보이는지, 상태 변화가 색으로 반영되는지 확인.
- `ros2 topic echo /diagnostics_agg`로 집계된 메시지 구조 확인 (나중에 앱 미러링 붙일 때 참고할 수 있게).
