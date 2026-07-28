# 앱↔로봇 상태 전이 핸드셰이크 프로토콜

이 문서는 **스펙**이다 — 앱(Android) 저장소가 이 머신에 없어서 실제 Kotlin
코드는 여기서 수정할 수 없다. 아래 계약대로 앱 쪽을 구현/수정해야 한다.

로봇 쪽 구현: `jangauto_mission/scripts/mission_state_machine.py`(판단, `/robot_status`
타입드 토픽 발행만) + `jangauto_mission/scripts/mission_diagnostics_monitor.py`
(`/diagnostics_agg`의 실제 문제를 `/jangauto_mission/error`로 연결) +
`jangauto_hmi/scripts/app_websocket_bridge.py`(웹소켓 중계 **겸** 앱 JSON 조립 —
`/robot_status`+`/odometry/global`+`/odom`+`/map`을 모아 직접 조립해서 WS로 내보낸다,
ROS 재발행 없음).

## 1. 개념 변화: "상태 지정"이 아니라 "전이 요청"

- 앱이 보내는 와이어 포맷 자체(`sw_bits` 등 JSON)는 바뀌지 않는다.
- 다만 의미가 바뀐다: 이전엔 "내가 원하는 상태를 계속 알려주는 것"이었다면,
  이제는 **"이 상태로 전이해달라는 요청 하나"**로 해석해야 한다.
- 로봇은 이 요청을 항상 수락하지 않는다 — 순서 규칙(아래)에 안 맞으면 거부한다.
- 주기적 재전송(기본 500ms)은 그대로 유지한다 — 하트비트 역할을 겸하며,
  끊기면 로봇이 자동으로 STOP으로 전이한다.

## 2. 상태 전이 순서 규칙 (로봇이 강제)

- STOP/KEY/CAL은 서로 언제든 자유롭게 오갈 수 있다.
- ALIGN은 STOP/KEY/CAL에서만 요청할 수 있다.
- RUN은 ALIGN에서만 요청할 수 있다(STOP/KEY/CAL에서 RUN 직행 불가).
- **내려가는 요청은 항상 자유**: RUN/ALIGN에서 STOP/KEY/CAL로, RUN에서
  ALIGN으로 가는 요청은 지금 상태가 뭐든 항상 수락된다.
- 위 규칙과 무관하게, 로봇 내부 에러나 명령 타임아웃 시 로봇은 스스로
  STOP으로 전이한다 — 이건 앱 요청과 무관하게 일어나므로 앱은 §3의
  상태 미러로만 알 수 있다.

## 3. 명령별 개별 응답 없음

예전에는 앱이 control_state 메시지를 하나 보낼 때마다 로봇이
`/app/control_state_ack`로 수락/거부 응답을 1개씩 돌려줬지만, 이제 그 채널
자체가 없다. `/robot_status`(§4)를 계속 주기적으로 내보내므로, 앱은
**요청 후 `mode`가 원하는 값으로 바뀌는지**를 보고 수락 여부를 판단해야 한다.

- 수락되면: 곧 `mode`가 요청한 값으로 바뀐 상태 미러가 도착한다.
- 거부되면: `mode`가 안 바뀐 채로 상태 미러가 계속 그대로 온다 — **거부 사유
  텍스트는 더 이상 전달되지 않는다.** 필요하면 앱이 자체적으로 "요청 후
  1~2초 내 mode 변화 없음 = 거부/무응답"으로 타임아웃 판단해야 한다.

## 4. 로봇 → 앱: 상태 미러 (JSON, WS로만 전송·ROS 토픽 없음)

`app_websocket_bridge.py`가 `/robot_status`(mode/in_error/error_reason) +
`/odometry/global`(위치) + `/odom`(속도)을 모아 **주기적으로**(기본
`app_status_publish_period_sec=0.2초`, 5Hz) 조립해서 웹소켓으로 보낸다 —
상태가 안 바뀌어도 계속 나가며, 이게 사실상 **로봇→앱 하트비트**를 겸한다
(앱→로봇 방향은 앱의 500ms 주기 전송이 이미 하트비트 역할을 함).

```json
{
  "mode": "STOP",
  "in_error": false,
  "error_reason": "",
  "tag_x": 1.23,
  "tag_y": 4.56,
  "tag_ori": 0.12,
  "tag_vel": 0.0,
  "tag_yaw_rate": 0.0
}
```

- `mode`/`in_error`/`error_reason`: 항상 포함(`/robot_status`를 한 번도 못 받은
  부팅 직후에는 아예 전송 안 함).
- `tag_x`/`tag_y`/`tag_ori`: `/odometry/global`(GPS+IMU 전역 EKF, map 프레임)
  출처 — 아직 수신 전이면 생략.
- `tag_vel`/`tag_yaw_rate`: `/odom`(IMU 로컬 EKF) 출처 — 아직 수신 전이면 생략.
- **`timestamp` 필드는 보내지 않는다** — 로봇 자체 시계로 찍으면 요청-응답
  구조가 아니라 폰과의 시계 오차가 섞여 의미 없는 "핑"이 된다는 걸 확인해서
  의도적으로 뺐다. 앱의 관련 핑 측정 코드는 계속 비활성 상태로 남는다(§5 참고).
- 웹소켓 접속 직후에도 마지막으로 조립된 값을 즉시 한 번 보내준다.
- `in_error`가 true면 앱 명령과 무관하게(에러/타임아웃으로) STOP이 강제된
  상태라는 뜻 — `error_reason`에 사유가 담긴다.

## 6. 로봇 → 앱: 지도 데이터 (`map_data`, 신뢰성 전송)

`app_websocket_bridge.py`가 `/map`(`nav_msgs/OccupancyGrid`)을 구독해서 점유 셀의 꼭짓점을
OpenCV(`findContours`+`approxPolyDP`)로 추출한 뒤, 앱이 이미 파싱 가능한 `map_data` 메시지의
**`anchors` 필드**로 보낸다:

```json
{"type": "map_data", "msg_id": "a1b2c3d4", "anchors": [{"x": -10.0, "y": -10.0}, ...]}
```

- **필드명이 의미상 안 맞는 것을 알고 있다** — 실제 UWB 앵커 위치가 아니라 `/map`에서 추출한
  장애물(현재는 placeholder 테두리) 꼭짓점을 임시로 `anchors`에 담은 것. `walls`
  (`List<List<Point>>`, 연결된 선으로 그려짐)가 구조적으로 더 맞지만, 앱 코드를 안 고치고도
  지금 당장 화면에 뭔가 뜨게 하려고 앱이 이미 아는 평평한 `List<Point>` 타입에 맞춘 임시
  조치다. 앱 쪽 정식 반영 항목은 §7 참고.
- `/map` 내용이 바뀔 때만(dedup) 새 메시지를 보낸다 — placeholder가 10Hz로 계속 오지만
  내용이 안 바뀌면 재전송하지 않는다.
- **신뢰성 전송**: 앱의 `MoveRequest`/`PoweroffRequest` 등과 동일한 msg_id+`AppAck` 패턴.
  `msg_id`를 받은 앱은 `{"type": "app_ack", "msg_id": "a1b2c3d4"}`로 확인 응답을 보내야 한다.
  응답이 `map_data_retry_timeout_sec`(기본 1.0초, 앱의 `RETRY_TIMEOUT_MS`와 동일) 안에 안 오면
  같은 `msg_id`로 재전송하고, `map_data_max_retries`(기본 3회, 앱의 `MAX_RETRIES`와 동일) 소진
  후 포기한다(`/diagnostics`의 'Map data delivery' 항목이 ERROR로 바뀜).
- 앱 쪽은 이미 `SocketManager.processRobotMessage()`에서 `type=="map_data"` 수신 시
  `AppAck(msgId=...)`을 자동으로 돌려주도록 구현돼 있어서(`model/RobotMessages.kt`), **앱 코드
  변경 없이 이 왕복이 이미 동작한다.**

## 7. 안드로이드 앱 변경 목록 (이번엔 앱 저장소를 직접 고치지 않음 — 향후 반영용)

1. **`ControlAck`/`control_state_ack` 파싱 제거**: `SocketManager.processRobotMessage()`의
   `requested_mode`+`accepted` 분기와 `model/RobotMessages.kt`의 `ControlAck`는
   로봇이 이제 절대 안 보내는 메시지 — 죽은 코드이므로 제거 대상.
2. **거부 사유 UX 재설계**: 지금은 `accepted:false` 수신 시 `feedbackListener`로
   `reason`을 토스트 등으로 띄우는데, 이 신호가 사라짐. 명령 전송 후 일정
   시간(예: 1~2초) 내 `mode`가 요청한 값으로 안 바뀌면 거부/무응답으로 간주하는
   타임아웃 기반 UX 필요(구체적 사유 텍스트는 없음).
3. **앱 코드 변경 없이 자동으로 살아나는 기능**: `tag_x`/`tag_y` 기반 이동 트레일
   (`CommandState.addHistory`), `map_data`/`anchors` 수신+`AppAck` 자동 응답
   (`SocketManager.setMapDataListener`, `GPathFragment`) — 로봇이 이제 이 필드들을
   채워 보내므로 기존 앱 코드가 그대로 동작 시작한다(§6).
4. **계속 비활성 상태로 남는 기능**: `RobotStatus.timestamp` 기반 핑 측정 —
   로봇이 `timestamp`를 아예 안 보내기로 했으므로(§4) 계속 null. 필요해지면
   로봇 자체 시계 대신 앱이 보낸 `control_state.timestamp`를 그대로 echo하는
   방식(양쪽 다 폰 시계 하나만 사용해 시계 동기화 문제 회피)으로 재검토.
5. **수신 빈도 변화**: 상태 미러 메시지가 5Hz로 계속 옴(전엔 상태
   변화 시에만). `processRobotMessage` 처리가 가벼워서 문제없어 보이지만,
   실제 기기에서 배터리/UI 갱신 빈도 확인 권장.
6. **`anchors` 필드 리네임/재구조화 필요**: §6에서 설명했듯 지금 `anchors`엔 UWB
   앵커가 아니라 `/map` 꼭짓점이 들어있다 — 나중에 필드명을 더 명확한 것(예:
   `obstacle_points`)으로 바꾸고, 필요하면 `walls`처럼 폴리곤 그룹 구조
   (`List<List<Point>>`)로 재구성해서 `MapView`가 흩어진 점이 아니라 연결된 선으로
   그리도록 앱 모델(`RobotMessages.kt`)·렌더링도 같이 손볼 것.
7. **이번엔 손 안 댐**: `MoveRequest`/`PoweroffRequest`/`GeneratePathRequest`/`MoveAck`
   관련 로직은 로봇 쪽에 처리 노드가 없는 채로 유지된다(다음 단계 과제) — 앱도
   지금은 이 부분을 안 건드려도 된다.
