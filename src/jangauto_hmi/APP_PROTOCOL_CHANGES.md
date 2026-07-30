# 앱 프로토콜 변경사항 — `calibration_complete` 필드 추가

로봇→앱 상태 메시지(WebSocket, `ws://<host>:8887`)에 `calibration_complete`
필드가 추가됐다. 앱 쪽에서 이 문서를 보고 대응하면 된다.

## 무엇이 바뀌었나

로봇이 `app_status_publish_period_sec` 주기로 계속 보내는 상태 메시지에
새 키 `calibration_complete`(bool)가 추가됐다. 기존 필드는 이름·의미
변경 없음 — 새 필드만 늘어난다.

**변경 전:**
```json
{
  "current_state": "KEY",
  "in_error": false,
  "error_reason": "",
  "tag_x": 1.23,
  "tag_y": 4.56,
  "tag_ori": 0.78,
  "tag_vel": 0.3,
  "tag_yaw_rate": 0.0
}
```

**변경 후:**
```json
{
  "current_state": "KEY",
  "in_error": false,
  "error_reason": "",
  "calibration_complete": true,
  "tag_x": 1.23,
  "tag_y": 4.56,
  "tag_ori": 0.78,
  "tag_vel": 0.3,
  "tag_yaw_rate": 0.0
}
```

(`tag_x`/`tag_y`/`tag_ori`/`tag_vel`/`tag_yaw_rate`는 위치/속도 추정치를
아직 못 받았을 때는 생략될 수 있음 — 기존과 동일, 이번 변경과 무관.)

## `calibration_complete`가 뜻하는 것

- CAL(캘리브레이션)을 이번 세션에서 한 번이라도 성공적으로 완료했는지.
- `false`로 시작해서, CAL이 한 번 성공하면 `true`로 바뀐 뒤 로봇 소프트웨어를
  재시작하기 전까지는 계속 `true`로 유지된다(다시 `false`로 안 돌아옴).

## 왜 앱이 이 값을 봐야 하나

`command: "move"`(MoveRequest)는 `calibration_complete`가 `false`인 동안
**항상 조용히 거부된다** — 개별 거부 사유 텍스트도 따로 안 온다. 지금까지는
앱이 이동을 시도해봐야만(반응 없음) 간접적으로 "아직 CAL을 안 했나보다"라고
유추할 수 있었다. 이제는 이 필드를 미리 보고:

- `calibration_complete`가 `false`인 동안 이동 관련 버튼/기능을 비활성화하거나
  "먼저 CAL을 완료하세요" 같은 안내를 보여줄 수 있다.
- `true`로 바뀌는 순간 UI를 활성화하면 된다.

## 하위호환성

기존에 이 필드를 모르고 파싱하던 앱 코드는 그대로 동작한다(추가 필드는
무시됨). 다만 앞으로는 이 필드가 항상 채워져서 나오므로, "CAL 완료 전
이동 시도가 반응이 없는" 것에 대한 임시방편(타임아웃 등)을 쓰고 있었다면
이 필드로 대체하는 걸 권장.
