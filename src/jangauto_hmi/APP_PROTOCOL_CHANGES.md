# 앱 프로토콜 변경사항 — ㄹ자 커버리지 경로 생성/선택

`app_websocket_bridge.py`가 노출하는 WebSocket(`ws://<host>:8887`) JSON 프로토콜에
추가된 부분만 정리한 문서. 기존 프로토콜(`move`, `map_data`, control_state 등)은
`app_websocket_bridge.py` 모듈 docstring 참고. **이 문서는 로봇 쪽 구현이 아니라
앱(모바일) 팀이 구현해야 할 스펙만 다룬다** — 앱 코드베이스는 이 저장소 밖에
있음.

## 좌표계

`polygon`/`waypoints`의 x,y는 모두 `map_data`에서 받은 것과 동일한 map 프레임
좌표(m)다. 즉 앱이 `map_data`로 그린 지도 위에 그대로 겹쳐서 다각형을 입력하고
경로를 그릴 수 있다.

## 게이팅 규칙

`generate_coverage_path`/`select_coverage_path`는 로봇의 `current_state`가
**STOP/KEY/CAL일 때만** 수락된다. ALIGN/RUN 상태(즉 실제 주행 중이거나
정렬 중)에는 거부된다 — 앱 UI는 이 두 명령을 STOP/KEY/CAL 상태에서만
활성화하는 게 자연스럽다. 상태값은 주기 상태 브로드캐스트(`current_state`
필드, 아래 참고)로 계속 내려온다.

## 1. 경로 생성 요청 — `generate_coverage_path`

**앱 → 로봇**
```json
{
  "command": "generate_coverage_path",
  "msg_id": "<앱이 발급한 요청 ID>",
  "polygon": [{"x": 0.0, "y": 0.0}, {"x": 40.0, "y": 3.0}, {"x": 45.0, "y": 28.0}, {"x": 30.0, "y": 33.0}, {"x": 5.0, "y": 30.0}],
  "edge_safety_dist": [0.5, 0.8, 0.6, 1.0, 0.5],
  "robot_radius": 0.4,
  "yaw_deg": 8.0,
  "ridge_spacing": 1.2,
  "headland_length": 2.0,
  "cell_size": 0.2
}
```
- `polygon`: 온실 외곽 다각형 꼭짓점(시계/반시계 무관, 3개 이상).
- `edge_safety_dist`: 변별 안전거리(m). **`polygon` 길이와 같아야 하며**, `i`번째
  값은 `polygon[i] -> polygon[i+1]` 변(마지막은 마지막 점->첫 점)에 대응한다.
- `robot_radius`: 로봇 풋프린트 외접원 반지름(m).
- `yaw_deg`: 두둑을 정렬할 기준 방향(deg).
- `ridge_spacing`: 두둑 간격(m).
- `headland_length`: 헤드랜드 길이(m, 양끝 각각).
- `cell_size`: 선택. 생략하거나 0이면 로봇 기본값(0.2m) 사용.

**로봇 → 앱 (1차 응답, 1회성, 재전송 없음)**
```json
{"type": "generate_coverage_path_ack", "msg_id": "<요청의 msg_id>", "accepted": true, "reason": ""}
```
`accepted=false`인 경우 `reason`에 사유(상태 불가/파라미터 오류/계산 실패 등
사람이 읽을 한글 문자열)가 담긴다. 계산 자체가 보통 1초 이내로 끝나므로, 이
ack은 "접수했다"가 아니라 "계산까지 마쳤다"는 뜻이다 — 성공 시에만 바로 뒤에
`coverage_path_result`가 온다.

**로봇 → 앱 (계산 결과, 중요 — 도달 보장)**
```json
{
  "type": "coverage_path_result",
  "msg_id": "<로봇이 새로 발급한 결과 ID>",
  "paths": [
    {
      "start_side": "left",
      "rect_length": 38.2, "rect_width": 24.5, "work_len": 34.2, "n_ridges": 20,
      "waypoints": [
        {"x": 1.2, "y": 3.4, "yaw": 0.14, "turn_angle": 0.0, "dist_to_next": 12.3, "kind": "start", "row_index": 0},
        {"...": "..."}
      ]
    },
    {"start_side": "right", "...": "..."}
  ]
}
```
- `paths`는 항상 2개(`paths[0]`=좌측 시작, `paths[1]`=우측 시작). 각 `waypoints`
  배열에 그 경로의 **전체 지점이 순서대로 다 들어있다** — 앱이 두 후보를 화면에
  그려서 사용자에게 보여주는 것과, 선택 후 그대로 주행하는 것 둘 다 이 배열
  하나로 충분하다.
- `kind`: `start`/`work_start`/`work_end`/`turn_out`/`turn_in`/`end` — 두둑 진입/작업/
  헤드랜드 이동/종료 구간을 구분(선을 다르게 그리고 싶을 때 참고).
- `rect_length`/`rect_width`/`work_len`/`n_ridges`: UI에 "N개 두둑, 작업길이 M m"
  같은 요약을 보여줄 때 쓰는 부가 정보(주행 자체엔 불필요).
- **재전송**: 앱이 `app_ack`을 안 보내면 최대 3회, 1초 간격으로 재전송한다.
  단, 아래 `select_coverage_path`를 이 `msg_id`로 보내면 그 자체가 수신 확인을
  겸하므로 별도 `app_ack`이 없어도 재전송이 멈춘다. `map_data`와 달리 재연결
  keepalive는 없다 — 이 결과는 1회성 제안이라, 오래된 제안을 나중에 다시
  들이미는 게 의미가 없기 때문.
- 새 `generate_coverage_path` 요청이 완료되면, 그 이전에 아직 미확인이던
  결과는 자동으로 대체(재전송 중단)된다 — 최신 요청 하나만 유효.

**앱 → 로봇 (선택 사항, 명시적 수신 확인)**
```json
{"type": "app_ack", "msg_id": "<coverage_path_result의 msg_id>"}
```

## 2. 경로 선택 — `select_coverage_path`

**앱 → 로봇**
```json
{"command": "select_coverage_path", "msg_id": "<coverage_path_result의 msg_id>", "path_index": 0}
```
- `msg_id`: 반드시 직전에 받은 `coverage_path_result`의 `msg_id`를 그대로 참조.
- `path_index`: `0`(좌측 시작) 또는 `1`(우측 시작).

**로봇 → 앱**
```json
{"type": "select_coverage_path_ack", "msg_id": "<위 요청의 msg_id>", "accepted": true, "reason": ""}
```
`accepted=false` 사유: 참조한 `msg_id`가 만료/불일치(이미 다른 결과로 대체됐거나
너무 오래돼 재전송이 끝난 경우), 상태가 STOP/KEY/CAL이 아님, `path_index`가
0/1이 아님.

선택이 수락되면 로봇은 그 경로를 내부에 저장해두고, **이후 앱이 sw_bits를
RUN으로 전환하면 그 경로를 그대로 주행한다.** 선택된 경로는 주행이 끝나도
지워지지 않는다 — 재선택 없이 같은 경로를 다시 RUN해도 그대로 재사용된다.
새로 `generate_coverage_path`→`select_coverage_path`를 다시 하면 그 경로로
교체된다.

## 3. 상태 브로드캐스트에 추가된 필드

기존 주기 상태 JSON(위치/속도/`current_state`/`calibration_complete` 등)에
`path_selected`(bool)가 추가됐다 — `calibration_complete`와 동일한 취지로,
"한 번이라도 경로를 선택했는가"를 앱이 미리 알 수 있게 한다(RUN 전환 전
UI 가드용).

## 시퀀스 요약

```
앱 -> generate_coverage_path
로봇 -> generate_coverage_path_ack (accepted)
로봇 -> coverage_path_result (후보 2개, 재전송 가능)
앱 -> select_coverage_path (path_index 선택)
로봇 -> select_coverage_path_ack (accepted)
앱 -> (sw_bits를 RUN으로) control_state
로봇 -> 선택된 경로를 Nav2로 주행
```
