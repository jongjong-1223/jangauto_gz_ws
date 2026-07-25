# jangauto_mission (YASMIN 상태머신) + diagnostics 파이프라인

## Context

`jangauto_mission` 패키지는 원래 완전히 빈 `ament_cmake` 스켈레톤이었다. `/state_command`/`/robot_state`라는 상태 토픽은 코드에 등장했지만, 그걸 쓰는 유일한 코드(`jangauto_hmi/scripts/reference/app_bridge.py`)는 **빌드되지 않는 참고용 파일**이라 "현재 시스템 상태가 뭔지 아는 노드"가 전혀 없었다 — 이 패키지가 그 자리를 채운다. 지금은 아래 내용 전부 구현·빌드·실행 검증·커밋 완료된 상태다.

실제로 돌고 있는 프로토콜은 `app_websocket_bridge.py`가 앱에서 받은 JSON을 그대로 재발행하는 `/app/control_state` (String, `{"sw_bits":16,...}` 형태)이다. 확정된 설계:
- FSM은 `/app/control_state`를 **구독**해서 `sw_bits` 값으로 STOP/KEY/CAL/ALIGN/RUN 5개 상태 사이 전이를 판단한다.
- **상태 전이는 앱 명령만이 아니라 시스템 내부 조건(에러)에 의해서도 일어난다.** 에러 소스는 전용 신설 토픽 `/jangauto_mission/error`(std_msgs/String, 빈 문자열=해제/비어있지 않으면 에러 사유) — 다른 어떤 노드든 여기 발행하면 FSM이 STOP으로 강제 전이한다.
- **상태 전이 순서 제약이 있다**: STOP/KEY/CAL은 서로 언제든 자유롭게 전이 가능하지만, ALIGN은 STOP/KEY/CAL에서만, RUN은 ALIGN에서만 갈 수 있다. 반대로 내려가는 전이(RUN→아래, ALIGN→아래)는 항상 자유다. ERROR/TIMEOUT에 의한 강제 STOP은 이 규칙과 무관하게 항상 즉시 적용된다.
- **앱↔로봇 핸드셰이크**: 앱이 전이를 요청하면 로봇이 수락/거부 + 보낸 값의 echo + 현재 진짜 상태를 답장한다(`/app/control_state_ack`). 앱은 별도 요청ID 없이 로봇이 알려주는 `current_mode`를 그대로 화면에 반영한다(로봇이 authoritative). 상태가 바뀔 때마다 앱 전용 미러(`/app/robot_status`)도 발행 — ERROR/TIMEOUT처럼 앱 명령이 아닌 이유로 바뀔 때도 앱이 알 수 있는 유일한 경로. 앱 쪽에 실제로 요구되는 변경사항은 `jangauto_hmi/scripts/reference/APP_PROTOCOL_HANDSHAKE.md`에 스펙으로 문서화했다(앱 저장소가 이 머신에 없어 코드는 직접 수정 불가).
- `cmd_vel` 게이팅(예전 `app_bridge.py`가 하던, KEY 상태에서만 이동명령 허용하는 로직)은 **범위 밖** — 별도 노드로 나중에.
- 상태머신 라이브러리는 **YASMIN** (`ros-jazzy-yasmin`, `-ros`, `-msgs`, `-viewer`).

**상태 브로드캐스트**: `jangauto_msg/msg/Status.msg`를 신규 정의해 `/robot_status` 토픽으로 발행한다(ROS-Industrial `industrial_msgs/RobotStatus` 등 업계 선례를 따름 — `mode` + 에러 여부를 구조화된 필드로):
```
std_msgs/Header header
string mode           # STOP, KEY, CAL, ALIGN, RUN
bool in_error
string error_reason   # in_error가 false면 빈 문자열
```
QoS는 `RELIABLE` + `TRANSIENT_LOCAL`(latched) + `KEEP_LAST(depth=1)` — 늦게 join하는 구독자도 최신 상태를 즉시 받도록. **알려진 한계**: 이건 소프트웨어 계층의 상태 전파이며, 하드웨어 긴급정지(릴레이 기반 등)를 대체하지 않는다.

**YASMIN 소스 레벨 조사로 확인한 것 (설계에 반영됨)**:
- `StateMachine.add_state(name, state, transitions={...})`로 상태·전이표를 정적으로 선언.
- `yasmin_ros.MonitorState`는 토픽 1개 전용이라 "앱 명령 + 에러, 두 소스 → 같은 목적지 전이"를 표현 못 함 — 커스텀 fan-in(`ControlAndErrorMonitor`)을 직접 구현해서 해결.
- `MonitorState`의 내장 `TIMEOUT` 메커니즘을 응용해 "명령 미수신 시 자동 STOP"을 별도 워치독 없이 구현.
- YASMIN은 전이 시 아무것도 자동 발행하지 않으므로(`yasmin_viewer`는 별개의 디버그용 시각화 채널), 모든 발행(`/robot_status`, `/app/robot_status`, `/app/control_state_ack`)은 State 코드 안에서 직접 처리.
- 5개 상태 각각에 별도 State 인스턴스가 필요함(같은 인스턴스를 여러 이름에 재사용하면 `yasmin_viewer`가 깨짐 — 실행 확인된 라이브러리 제약) — 로직은 공유 `ControlAndErrorMonitor` 하나에만 두고, 상태별 얇은 어댑터(`ControlAndErrorMonitorState`)를 5개 만드는 구조.
- `YasminViewerPub`의 실제 생성자 인자 순서는 `(fsm, fsm_name, ...)` — 설치된 버전의 독스트링 Args 설명 순서가 반대로 적혀 있는 라이브러리 쪽 문서 버그이니 주의.

두 번째로, 전체 노드를 `/diagnostics`로 모니터링하는 파이프라인도 구축했다. 커스텀 파이썬 노드 2개(`app_websocket_bridge`, `gps_covariance_filler`)에 `diagnostic_updater`를 붙이고, `diagnostic_aggregator`로 집계해 `rqt_robot_monitor`로 확인한다. (나중에 `app_websocket_bridge`가 `/diagnostics_agg`를 구독해서 앱으로 미러링할 계획을 염두에 두고 이름/그룹 구조를 잡음.)

## Part 1 — `jangauto_msg`: `Status.msg` — 완료

`jangauto_msg/msg/Status.msg` 작성 완료, `jangauto_msg/CMakeLists.txt`의 `set(msg_files ...)`에 추가 완료.

## Part 2 — `jangauto_mission`: YASMIN 상태머신 — 완료

### 패키지 전환 — 완료
`ament_cmake` + `install(PROGRAMS scripts/*.py ...)` 패턴(`jangauto_gps_driver/CMakeLists.txt` 참고). `package.xml`에 `rclpy`, `std_msgs`, `jangauto_msg`, `yasmin`, `yasmin_ros`, `yasmin_msgs`, `yasmin_viewer` 추가 완료. yasmin 계열은 사용자가 `sudo apt-get install ros-jazzy-yasmin ros-jazzy-yasmin-ros ros-jazzy-yasmin-msgs ros-jazzy-yasmin-viewer`로 수동 설치 완료.

### `jangauto_mission/scripts/mission_state_machine.py` — 완료
- `sw_bits` → 상태 이름 매핑: `16→STOP`, `8→KEY`, `4→CAL`, `2→ALIGN`, `1→RUN`.
- `ControlAndErrorMonitor`(공유 몸통, 인스턴스 1개) + `ControlAndErrorMonitorState`(상태별 얇은 어댑터, 인스턴스 5개) 구조.
- `ALLOWED_TARGETS` 딕셔너리로 상태별 허용 목표 모드를 정의(위 Context의 순서 제약). 상태마다 outcome 목록/전이표를 이로부터 동적으로 구성 — 예전엔 5개 상태가 완전연결이라 동일한 `transitions`를 공유했지만 이제 상태별로 다름.
- 앱 명령으로 결정된 outcome은 `f"APP_TO_{target}"` 형태로 이름 붙여서 "앱에서 온 명령임"을 명시(예: `"APP_TO_RUN"`). ERROR/TIMEOUT은 그대로.
- 순서 규칙에 안 맞는 요청은 YASMIN 전이를 발생시키지 않고(`continue`) 거부 ack만 발행.
- `timeout`/`maximum_retry`(5초 × 2회) 초과 시 `"TIMEOUT"` outcome → STOP.
- 상태가 바뀔 때(이전과 다를 때만) `/robot_status`(jangauto_msg/Status, RELIABLE+TRANSIENT_LOCAL+depth1) + `/app/robot_status`(String JSON 미러) 동시 발행.
- 앱 명령 1건 처리마다 `/app/control_state_ack`(String JSON: `sw_bits`/`requested_mode`/`accepted`/`current_mode`/`reason`) 발행.
- `yasmin_viewer.YasminViewerPub`으로 FSM 시각화(디버그 채널).

### `jangauto_mission/launch/mission.launch.py` — 완료
`mission_state_machine.py` 노드 하나를 실행하는 얇은 launch 파일.

### `jangauto_bringup` 연동 — 완료
`tracked_v3.launch.py`에 mission include 추가, `jangauto_bringup`의 `CMakeLists.txt`/`package.xml`에 `jangauto_mission` 의존성 추가.

## Part 2.5 — `jangauto_hmi/scripts/app_websocket_bridge.py`: 앱 핸드셰이크 중계 — 완료

`/app/robot_status`, `/app/control_state_ack` 구독 추가. rclpy 콜백 스레드에서 `asyncio.run_coroutine_threadsafe(...)`(기존 `_stop_ws_server()`가 쓰던 패턴 재사용)로 안전하게 넘겨서 연결된 모든 웹소켓 클라이언트에 그대로 전달. `/app/robot_status`는 마지막 값을 캐시해뒀다가 새 클라이언트 접속 즉시 한 번 보내줌(`/robot_status`의 latched 동작을 웹소켓 레이어에서도 재현). 이 브릿지는 여전히 메시지 내용을 해석하지 않는 "덤(dumb) 중계"로만 남는다 — accept/reject 판단이나 시퀀싱 로직은 전부 `mission_state_machine.py` 책임.

**실측 중 발견/수정한 버그**: `/app/robot_status`를 처음엔 기본 QoS(volatile)로 발행했는데, `app_websocket_bridge.py`가 `mission_state_machine.py`보다 늦게 뜨면(또는 재시작하면) 그 사이의 마지막 상태를 영영 못 받아 캐시가 계속 비어있고, 신규 웹소켓 접속자에게 즉시 보내줄 값이 없는 문제가 실제로 재현됐다. `/app/robot_status` 발행 QoS를 `/robot_status`와 동일하게 RELIABLE+TRANSIENT_LOCAL로 바꾸는 것만으론 부족했고, **구독 쪽(`app_websocket_bridge.py`)도 같은 durability로 맞춰야** late-join 시 실제로 과거 값을 받는다는 걸 확인했다(QoS가 호환되는 것과, 늦은 구독자가 과거 값을 받는 것은 별개 — 구독자가 VOLATILE을 요청하면 호환은 되지만 과거 값 재생은 안 해줌). 웹소켓 테스트 클라이언트로 재현 후 수정 완료.

앱(Android) 쪽에 실제로 필요한 변경사항은 `jangauto_hmi/scripts/reference/APP_PROTOCOL_HANDSHAKE.md`에 프로토콜 스펙으로 문서화했다 — 앱 저장소가 이 머신에 없어(전체 파일시스템 검색으로 확인) 코드 자체는 여기서 수정할 수 없다.

## Part 3 — 커스텀 노드 2개에 diagnostic_updater 추가 — 완료

### `jangauto_hmi/scripts/app_websocket_bridge.py`
`diagnostic_updater.Updater(self)` 생성, WS 서버 리슨 여부/mDNS 등록 상태/`self._ws_last_link_alive`로 OK/WARN/ERROR 판정. `jangauto_hmi/package.xml`에 `diagnostic_updater` exec_depend 추가.

### `jangauto_drivers/jangauto_gps_driver/scripts/gps_covariance_filler.py`
같은 패턴 — GPS 데이터 최근성으로 OK/WARN(끊김)/ERROR(수신 이력 없음). `jangauto_gps_driver/package.xml`에 exec_depend 추가.

## Part 4 — `diagnostic_aggregator` 연동 — 완료

`jangauto_bringup/config/diagnostic_analyzers.yaml`(hmi/gps 두 그룹), `jangauto_bringup/launch/diagnostics.launch.py` 작성, `tracked_v3.launch.py`에 include. YAML은 `/**: ros__parameters: analyzers: ...` 구조(실제 `diagnostic_aggregator` 소스의 파라미터 프리픽스 규칙 확인 후 작성). 사용자가 `sudo apt-get install ros-jazzy-diagnostic-aggregator ros-jazzy-rqt-robot-monitor` 설치 완료.

## 실행 순서 (전부 완료)

1. `jangauto_msg`에 `Status.msg` 추가
2. `jangauto_mission` 패키지 전환 + `mission_state_machine.py`(순서 제약, outcome 이름, 핸드셰이크 포함) + `mission.launch.py`
3. `app_websocket_bridge.py`(diagnostic_updater + 핸드셰이크 중계), `gps_covariance_filler.py`(diagnostic_updater)
4. `diagnostic_analyzers.yaml` + `diagnostics.launch.py`
5. `tracked_v3.launch.py`에 mission + diagnostics include, 의존성 정리
6. apt 패키지 설치(yasmin 계열, diagnostic_aggregator, rqt_robot_monitor) + 시스템 패키지 업그레이드(fastcdr ABI 불일치 해결)
7. `APP_PROTOCOL_HANDSHAKE.md` 작성

## 검증 (실측 완료 항목 포함)

- STOP→KEY→CAL→ALIGN→RUN 순서 전이, 시작 시 TIMEOUT→STOP, `/jangauto_mission/error` → 즉시 STOP(어느 상태에서든), 에러 해제 후 정상 명령으로 복구 — 전부 `ros2 topic pub`/`ros2 topic echo`로 실측 확인.
- `/robot_status` QoS가 RELIABLE/TRANSIENT_LOCAL임을 `ros2 topic info --verbose`로 확인, 늦게 붙은 구독자도 즉시 latched 값 수신 확인.
- `gps_covariance_filler`: 데이터 없음→ERROR, 수신→OK, 4초 idle→WARN(stale) 실측 확인.
- `diagnostic_aggregator` + `diagnostic_analyzers.yaml`: GPS 그룹이 실제 상태를 정확히 집계, 미기동 노드는 STALE로 표시됨을 확인.
- 순서 제약: STOP에서 RUN 직행 시도 → `/app/control_state_ack`에 `accepted:false` 확인. STOP→CAL→ALIGN→RUN 정상 경로는 각 단계 `accepted:true` 확인. RUN→STOP 직접 하강도 즉시 수락됨을 확인.
- 핸드셰이크 웹소켓 end-to-end: 테스트 클라이언트(`ws_test_client.py` 패턴)로 접속해서 `/app/robot_status`·`/app/control_state_ack`가 실제 웹소켓으로 중계되는지, 신규 접속 시 마지막 상태를 즉시 받는지(위 QoS 버그 수정 후) 전부 실측 확인.
- `app_websocket_bridge`: 이 머신에 없던 `python3-websockets`/`zeroconf`를 pip으로 설치해 라이브 테스트 완료(이전엔 미설치로 보류했었음).
