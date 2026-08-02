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
      "first_row_side": "near",
      "rect_length": 38.2, "rect_width": 24.5, "work_len": 34.2, "n_ridges": 20,
      "start_headland_corners": [{"x": 1.2, "y": 0.0}, {"x": 2.7, "y": 0.0}, {"x": 2.7, "y": 24.5}, {"x": 1.2, "y": 24.5}],
      "far_headland_corners": [{"x": 37.7, "y": 0.0}, {"x": 39.2, "y": 0.0}, {"x": 39.2, "y": 24.5}, {"x": 37.7, "y": 24.5}],
      "waypoints": [
        {"x": 1.2, "y": 3.4, "yaw": 0.14, "turn_angle": 0.0, "dist_to_next": 12.3, "kind": "start", "row_index": 0},
        {"...": "..."}
      ]
    },
    {"first_row_side": "far", "...": "..."}
  ]
}
```
- `paths`는 항상 2개(`paths[0]`=near, `paths[1]`=far). **둘 다 같은 출발 헤드랜드
  에서 같은 방향(+yaw)으로 첫 이동을 시작하고, 어느 두둑 줄부터 도는지만
  다르다**(near=가까운 줄부터, far=먼 줄부터) — 예전의 "좌/우 시작"(반대 방향
  출발) 개념이 아니다. 각 `waypoints` 배열에 그 경로의 **전체 지점이 순서대로
  다 들어있다** — 앱이 두 후보를 화면에 그려서 사용자에게 보여주는 것과,
  선택 후 그대로 주행하는 것 둘 다 이 배열 하나로 충분하다.
- `kind`: `start`/`work_start`/`work_end`/`turn_out`/`turn_in`/`end` — 두둑 진입/작업/
  헤드랜드 이동/종료 구간을 구분(선을 다르게 그리고 싶을 때 참고).
- `rect_length`/`rect_width`/`work_len`/`n_ridges`: UI에 "N개 두둑, 작업길이 M m"
  같은 요약을 보여줄 때 쓰는 부가 정보(주행 자체엔 불필요).
- `start_headland_corners`/`far_headland_corners`: 헤드랜드 사각형 2개(출발쪽/
  반대쪽)의 꼭짓점 4개씩(map 프레임 좌표). **두 후보(`paths[0]`/`paths[1]`)에서
  값이 동일** — 헤드랜드 사각형 자체는 어느 줄부터 도는지와 무관하게 하나로
  고정되기 때문. 폴리곤으로 그대로 그리면 됨.
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
{"command": "select_coverage_path", "msg_id": "<이 요청 자체의 새 ID>", "ref_msg_id": "<coverage_path_result의 msg_id>", "path_index": 0}
```
- `msg_id`: 다른 명령(`move`, `generate_coverage_path` 등)과 동일하게, 이 요청
  자체를 추적하기 위해 **앱이 매번 새로 발급**하는 ID. ack의 `msg_id`로 그대로
  돌아온다.
- `ref_msg_id`: **직전에 받은 `coverage_path_result`의 `msg_id`를 그대로 참조**
  — "어떤 생성 결과 중에서 고르는지"를 가리키는 값이라 새로 만들면 안 된다.
  (`msg_id`/`ref_msg_id`를 분리한 이유: 다른 명령처럼 `msg_id`를 매번 새로
  만드는 습관과 충돌하지 않게 하기 위함 — 참조가 필요한 필드만 이름을 다르게
  둠.)
- `path_index`: `0`(near, 가까운 줄부터) 또는 `1`(far, 먼 줄부터).

**로봇 → 앱**
```json
{"type": "select_coverage_path_ack", "msg_id": "<요청의 msg_id>", "accepted": true, "reason": ""}
```
`accepted=false` 사유: 참조한 `ref_msg_id`가 만료/불일치(이미 다른 결과로
대체됐거나 너무 오래돼 재전송이 끝난 경우), 상태가 STOP/KEY/CAL이 아님,
`path_index`가 0/1이 아님.

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

## 4. ALIGN 상태 의미 변경 — no-op → 실제 주행

- 과거: ALIGN은 진입 즉시 무조건 성공하는 더미(placeholder)였다.
- 현재: **선택된 경로의 첫 웨이포인트(`waypoints[0]`, kind=`start`)까지 Nav2로
  실제로 이동한다.** 거리에 따라 수십 초~분 단위로 소요될 수 있다.
- 앱 영향:
  - ALIGN 진입 전 반드시 `select_coverage_path`로 경로를 먼저 선택해야 한다.
    선택 없이 ALIGN에 진입하면 즉시 실패해 `STOP`으로 떨어지고(`in_error`),
    이 에러는 로봇을 재시작해야 풀리는 영구 상태다 — 앱은 경로 미선택 상태에서
    ALIGN(sw_bits)을 보내지 않도록 UI에서 막아야 한다(`path_selected` 필드,
    3번 참고).
  - **ALIGN 진입도 CAL이 한 번이라도 성공하기 전에는 거부된다** — 캘리브레이션
    완료 전 ALIGN 요청은 조용히 무시되며(`current_state`가 안 바뀜), 별도 에러
    응답은 없다.
  - ALIGN 체류 시간이 길어지므로 진행 중임을 보여주는 UI가 필요할 수 있다.
  - ALIGN 이동 중 다른 모드를 요청하면 Nav2 이동이 정상적으로 취소되고
    로봇이 즉시 멈춘다(에러 아님, 그냥 STOP으로 전이).
  - ALIGN 성공 후 sw_bits를 ALIGN으로 유지하면(self-loop) 재이동 없이 대기만
    한다.

## 5. RUN 동작 변경 — 시작점은 더 이상 RUN이 안 감

- RUN은 이제 `waypoints[0]`(시작점, ALIGN이 이미 이동을 담당)을 제외한
  `waypoints[1:]`부터 주행한다.
- **`coverage_path_result`의 `waypoints` 배열 자체는 그대로다 — JSON 스키마
  변경 없음.** 순수하게 로봇 측 실행이 ALIGN(첫 지점)과 RUN(나머지)으로
  나뉘는 것뿐이라, 앱이 진행률을 자체 계산 중이라면 "RUN의 진행 인덱스는
  전체 `waypoints` 배열 기준 1번 인덱스부터 시작한다"는 점만 반영하면 된다.

## 시퀀스 요약

```
앱 -> generate_coverage_path
로봇 -> generate_coverage_path_ack (accepted)
로봇 -> coverage_path_result (후보 2개, 재전송 가능)
앱 -> select_coverage_path (path_index 선택)
로봇 -> select_coverage_path_ack (accepted)
앱 -> (sw_bits를 ALIGN으로) control_state   # CAL 완료 필요, 경로 선택 필요
로봇 -> 선택된 경로의 시작 웨이포인트까지 Nav2로 이동
앱 -> (sw_bits를 RUN으로) control_state     # ALIGN에서만 진입 가능
로봇 -> 나머지 웨이포인트를 Nav2로 주행
```

## 변경 이력

### 2026-08-01: `select_coverage_path`에 `ref_msg_id` 필드 신설
- **배경**: 시뮬레이션에서 앱과 실제 연동 검증을 하던 중, `select_coverage_path`가
  로봇에 정상 도착해도 매번 거부되는 문제가 발견됨.
- **원인**: 앱이 `msg_id`에 이 요청 자신을 추적하기 위한 새 ID를 매번 생성해서
  보내고 있었음. 그런데 로봇 쪽 검증은 이 필드가 직전에 받은
  `coverage_path_result`의 `msg_id`를 그대로 참조해야 매칭되도록 설계돼 있었음
  — 다른 명령(`move`, `generate_coverage_path` 등)은 전부 `msg_id`가 "이 요청
  자신의 추적용 ID"인데 `select_coverage_path`만 예외적으로 "직전 결과를
  가리키는 참조값"이었던 게 혼란의 근본 원인.
- **변경**: `select_coverage_path`에 `ref_msg_id`(직전 `coverage_path_result`의
  `msg_id`를 그대로 참조) 필드를 신설. `msg_id`는 다른 명령들과 동일하게 앱이
  매번 새로 발급하는 자기 추적/ack용 ID로 되돌림 — 필드 하나가 명령마다 다른
  의미를 갖지 않도록 분리.
- **앱 측 반영 필요**: `SelectCoveragePathRequest`에 `ref_msg_id` 필드를
  추가하고, 경로 선택 시 `CommandState.lastResultMsgId`(직전
  `coverage_path_result` 수신 시 저장해둔 값)를 그대로 채워 보내야 함.
  `msg_id`는 기존처럼 새로 생성해서 보내면 됨.
- 부수적으로, 검증 과정에서 앱의 재연결 로직(`SocketManager.disconnectInternal()`
  이후 바로 `connect()`)이 이전 연결의 정상 종료를 기다리지 않아 일시적으로
  소켓 2개가 동시에 열리는 현상도 함께 발견됨 — 로봇 쪽 변경은 아니지만 앱
  팀에 별도 공유 필요.

### 2026-08-02: 경로 후보 재정의(near/far) + 헤드랜드 코너 전달 + ALIGN 실주행화
- **배경**: 경로 후보 2개를 "좌/우 반대 방향 출발"이 아니라 "같은 방향(+yaw)
  출발, 어느 두둑 줄부터 도는지만 다름"으로 바꿔달라는 요청. 앱이 헤드랜드
  영역을 화면에 그릴 수 있도록 좌표도 같이 필요. ALIGN을 실제 "경로 시작점
  으로 이동"하는 기능으로 채우면서 RUN과 역할을 분담.
- **변경**: `coverage_path_result.paths[]`의 `start_side`("left"/"right") 필드
  삭제 → `first_row_side`("near"/"far") 신설(1번 섹션 참고). `start_headland_corners`
  / `far_headland_corners` 필드 신설. ALIGN이 실제 Nav2 주행을 수행하도록 변경
  (4번 섹션). RUN이 `waypoints[0]`을 건너뛰도록 변경(5번 섹션).
- **앱 측 반영 필요**: `start_side` 파싱 제거 → `first_row_side` 파싱 추가(값
  의미 변경 포함, UI 라벨도 "좌/우 시작" → "가까운 줄/먼 줄부터"로 교체),
  헤드랜드 코너 2개 배열 렌더링 추가, ALIGN 대기 UI 추가, `select_coverage_path`
  가 ALIGN 진입보다 항상 먼저 오도록 흐름 강제, CAL 미완료 시 ALIGN 무반응
  케이스에 대한 안내.
- 시뮬레이션(Gazebo+Nav2) 전체 스택에서 앱으로 직접 CAL→경로 선택→ALIGN→RUN
  전 과정, ALIGN/RUN 취소, ALIGN self-loop까지 실증 검증 완료.
