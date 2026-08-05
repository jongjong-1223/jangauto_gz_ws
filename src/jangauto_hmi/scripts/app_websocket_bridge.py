#!/usr/bin/env python3
"""앱 웹소켓 브릿지 노드.

## 역할
- 모바일 앱과 단일 양방향 WebSocket(`ws://<host>:8887`)으로 연결한다.
- **앱→로봇(인바운드)**: 앱이 보낸 JSON을 그대로 ROS 토픽으로 재발행한다
  (`/app/control_state`, `/app/command`) — 내용을 해석하지 않는 덤 중계.
- **로봇→앱(아웃바운드, 상태)**: `/robot_status`(jangauto_msg/Status),
  `/odometry/global`(위치), `/odom`(속도), `/jangauto_mission/calibration_complete`
  (CAL 완료 여부) 네 ROS 토픽을 구독해서 앱이 기대하는 JSON을 이 노드가
  직접 조립한 뒤 주기적으로(`app_status_publish_period_sec`) WebSocket으로
  브로드캐스트한다 — ROS 토픽으로 재발행하지 않고 바로 WS로만 나간다. 앱
  JSON 스키마(필드 이름 등)에 대한 지식은 이 노드에만 있다. 명령별 개별
  응답(ack)은 없음 — `/robot_status`의 `current_state` 변화만으로 앱이
  수락 여부를 판단한다(거부 사유 텍스트는 전달하지 않음). 단
  `calibration_complete`는 MoveRequest가 항상 조용히 거부되는 이유(CAL 미완료)를
  앱이 미리 알 수 있도록 상태 JSON에 직접 포함한다.
- **로봇→앱(아웃바운드, 지도)**: `/global_costmap/costmap`(nav_msgs/OccupancyGrid —
  정적 `/map`+뎁스카메라 장애물+inflation이 합쳐진 Nav2 global costmap)을
  구독해서 `map_data` 메시지 하나로 두 가지를 같이 보낸다 — `map`(그리드 전체
  4개 모서리 좌표, 점유 여부와 무관한 순수 경계)과 `obstacles`(점유 셀
  컨투어를 OpenCV로 추출해 컨투어별로 묶은 꼭짓점 리스트의 리스트,
  `List<List<Point>>` — `map_occupied_threshold`(기본 99)로 진짜 장애물+
  inscribed radius 경계까지만 잡고, 순수 inflation 그라디언트는 제외).
  앱의 `MoveRequest` 등과 같은 msg_id+`AppAck` 신뢰성 프로토콜로 보내고,
  응답 없으면 주기적으로 재전송한다(`map_data_retry_timeout_sec`/
  `map_data_max_retries`). 내용이 안 바뀌면 새 msg_id로는 안 나가지만,
  마지막으로 보낸 JSON은 캐싱해뒀다가 신규 클라이언트 접속 시 즉시,
  그리고 `map_data_keepalive_period_sec`마다 저주파수로 재방송한다 —
  그 순간 접속이 없었거나 메시지를 놓친 클라이언트도 결국 지도를 받게
  하기 위함.
- **앱→로봇(아웃바운드 아님, MoveRequest)**: `/app/command`에 `command=="move"`가
  오면(다른 커맨드는 여전히 무시) 이 노드가 직접 판단한다 — CAL을 한 번이라도
  완료했고(`/jangauto_mission/calibration_complete`) 현재 KEY/CAL 모드일 때만
  수락, Nav2(`navigate_to_pose` 액션)에 목표 지점 goal을 보낸다. 거부 시
  사유를, 수락/거부 결과를 `move_ack`로 WS에 바로 응답한다(ROS 재발행 없음,
  판단·Nav2 연동·WS 응답까지 이 노드가 전담 — mission 쪽에 별도 노드 없음).
  Nav2 액션 클라이언트의 블로킹 호출이 WS 스레드를 막지 않도록 전용
  스레드풀(`ThreadPoolExecutor`)에서 처리한다. KEY 모드에서 조이스틱을
  건드리면(`cmd_vel_manual` 수신) 진행 중이던 move goal은 즉시 취소된다
  (사람의 직접 개입이 자율 이동보다 우선).
- **앱→로봇(ㄹ자 커버리지 경로)**: `command=="generate_coverage_path"`가 오면
  현재 STOP/KEY/CAL 상태일 때만 `generate_coverage_path` 액션(jangauto_mission의
  `coverage_path_action_server.py`)을 호출해 좌/우측 시작 후보 2개를 계산시키고,
  완료되면 `generate_coverage_path_ack`(수락/거부)와 `coverage_path_result`(후보
  2개, `map_data`와 동일한 msg_id+재전송+`app_ack` 신뢰성 패턴)를 함께 보낸다.
  `command=="select_coverage_path"`가 오면(직전 `coverage_path_result`의 msg_id를
  `ref_msg_id`로 참조 + `path_index`) 해당 경로를 `/jangauto_mission/selected_coverage_path`
  (latched)로 publish하고 `select_coverage_path_ack`를 응답한다 — 이 select
  자체가 `coverage_path_result`에 대한 ack도 겸한다(별도 `app_ack` 불필요).
  RUN 상태 진입 시 `run_action_server.py`가 이 latched 토픽의 최신값을 그대로
  주행한다.
- mDNS(`_robot._tcp.local.`)로 서비스를 광고해서 앱이 IP를 몰라도 자동
  탐색할 수 있게 한다.
- 하트비트(`/app/link_alive`)로 앱 연결 생존 여부를 추적하고,
  `diagnostic_updater`로 서버 상태·앱 연결·업스트림 토픽 수신 최근성·
  map_data 전송 성공 여부를 `/diagnostics`에 보고한다.

## `/app/*` 토픽 규칙
인바운드 두 토픽(`/app/control_state`, `/app/command`)은 이 노드만 발행하고
`mission_state_machine.py` 등 다른 노드는 구독만 한다. 아웃바운드
(`robot_status`)는 ROS 토픽 자체가 없다 — 조립된 JSON은 WS로만 나간다.

## 스코프 밖
JSON을 실제 로봇 명령(`cmd_vel`, poweroff, generate_path 등)으로 번역하는
로직은 여기 없다 — `app_bridge.py`/`app_wifi_rx.py`/`app_wifi_tx.py`(구
HTTP 프로토콜용 참고 코드, 현재 미사용)가 하던 역할이지만 이 노드는 그걸
가져다 쓰지 않는다. 지금 프로토콜(단일 WebSocket, 0-padding 문자열 대신
정수 비트필드)이 다르기 때문에 새로 만들었다.
"""
import asyncio
import concurrent.futures
import json
import math
import socket
import threading
import time
import uuid

import cv2
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Point32

from jangauto_msg.msg import Status, CoveragePath
from jangauto_msg.action import GenerateCoveragePath
from nav2_msgs.action import NavigateToPose

import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus

import websockets
from websockets.exceptions import ConnectionClosed

from zeroconf import Zeroconf, ServiceInfo

# Keys present in the periodic control-state payload the app sends every
# Config.TX_PERIOD_MS (500 ms by default on the app side).
CONTROL_STATE_KEYS = {'sw_bits', 'key_bits', 'speed_bits', 'video_bit', 'safe_bit'}

# 앱용 JSON을 조립하려고 구독하는 ROS 내부 소스들 — mission_state_machine.py(상태
# 판단), jangauto_perception(로컬라이제이션)이 각자 발행하는 타입드 토픽과,
# jangauto_uwb_driver(/map)+뎁스카메라 장애물이 합쳐진 nav2 global costmap이다.
ROBOT_STATUS_TOPIC = '/robot_status'          # mode/in_error/error_reason (jangauto_msg/Status)
ODOMETRY_GLOBAL_TOPIC = '/odometry/global'    # GPS+IMU 전역 EKF(map 프레임) — 위치(tag_x/y/ori) 출처
ODOMETRY_LOCAL_TOPIC = '/odom'                # IMU 로컬 EKF(odom 프레임) — 속도(tag_vel/yaw_rate) 출처
MAP_TOPIC = '/global_costmap/costmap'         # OccupancyGrid — 정적 /map+뎁스카메라 장애물+
                                               # inflation이 합쳐진 Nav2 global costmap, map_data(map/obstacles) 추출 출처
CALIBRATION_COMPLETE_TOPIC = '/jangauto_mission/calibration_complete'  # MoveRequest 수락 조건
CMD_VEL_MANUAL_TOPIC = 'cmd_vel_manual'       # 조이스틱 출력 — 값이 오면 진행 중인 move goal 취소

# MoveRequest가 수락되려면 이 모드들 중 하나여야 함(cmd_vel_arbiter.py의
# MODE_TO_SOURCE_TOPICS에서 KEY/CAL 둘 다 cmd_vel_nav_out을 허용하는 것과 짝을 이룸).
MOVE_ALLOWED_MODES = {'KEY', 'CAL'}
NAV2_ACTION_NAME = 'navigate_to_pose'
NAV2_WAIT_FOR_SERVER_TIMEOUT_SEC = 5.0

# generate_coverage_path/select_coverage_path가 수락되려면 이 모드들 중 하나여야
# 함 — RUN/ALIGN 중엔 계산 자원을 뺏기지 않고, 주행 중인 경로가 바뀌는 레이스도
# 막는다(coverage_path_action_server.py의 goal_callback도 동일 집합으로 게이팅).
COVERAGE_PATH_ALLOWED_MODES = {'STOP', 'KEY', 'CAL'}
GENERATE_COVERAGE_PATH_ACTION_NAME = 'generate_coverage_path'
SELECTED_COVERAGE_PATH_TOPIC = '/jangauto_mission/selected_coverage_path'

# 진단 최근성 판정 임계값(초) — gps_covariance_filler_simul.py와 동일한 관례
# (time.monotonic() 기반, 없음=ERROR/오래됨=WARN/정상=OK).
MAP_STALE_TIMEOUT_SEC = 3.0
ROBOT_STATUS_STALE_TIMEOUT_SEC = 3.0
ODOM_STALE_TIMEOUT_SEC = 3.0

# Must match Config.SERVICE_TYPE ("_robot._tcp.") in the app's NsdHelper.kt,
# with the ".local." domain that Android's NsdManager appends implicitly but
# python-zeroconf requires spelled out.
MDNS_SERVICE_TYPE = '_robot._tcp.local.'


def _quaternion_to_yaw(q) -> float:
    """geometry_msgs/Quaternion -> yaw(rad). 로봇 진행방향(tag_ori)을 앱에 보내려고
    평면 회전각 하나만 뽑아낸다(롤/피치는 지상 주행 로봇이라 무시)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _extract_obstacle_contours(msg: OccupancyGrid, occupied_threshold: int) -> list:
    """OccupancyGrid -> 점유 셀 컨투어별 꼭짓점 리스트의 리스트
    ([[{'x':..,'y':..}, ...], ...]) — `map_data`의 `obstacles` 필드용.

    - occupied_threshold 이상인 셀만 255로 이진화한 뒤, 1px 제로 패딩을 씌워서
      cv2.findContours에 넘긴다 — 지도 경계처럼 이미지 가장자리에 붙는 도형이
      패딩 없이는 잘려서 컨투어가 안 잡히는 OpenCV의 흔한 함정을 피하기 위함.
    - RETR_CCOMP + `hierarchy[i][3] == -1` 필터로 최상위(부모 없는) 컨투어만
      골라낸다 — 실측 확인된 RETR_EXTERNAL의 함정: 테두리(링) 모양처럼 구멍이
      있는 도형의 그 구멍 안에 별개로 떨어져 있는 장애물(예: 지도 테두리
      안쪽의 독립된 박스)을, 위상적으로는 둘 다 최상위 외곽선인데도
      RETR_EXTERNAL이 누락시키는 경우가 있다(OpenCV의 알려진 동작). approxPolyDP로
      각 컨투어의 꼭짓점 개수를 줄인다.
    - origin이 무회전이라고 가정(현재 이 프로젝트의 모든 OccupancyGrid 발행자가
      만족하는 조건)하고 셀 중심 좌표를 world 좌표(m)로 변환한다.
    - 컨투어(=개별 장애물)마다 꼭짓점 리스트를 따로 유지한다 — 앱이 각 장애물을
      연결된 선(폴리곤)으로 그릴 수 있어야 하므로 여기서 평탄화하지 않는다.
    """
    width = msg.info.width
    height = msg.info.height
    if width == 0 or height == 0:
        return []

    grid = np.array(msg.data, dtype=np.int16).reshape((height, width))
    binary = np.where(grid >= occupied_threshold, 255, 0).astype(np.uint8)
    padded = np.pad(binary, pad_width=1, mode='constant', constant_values=0)

    contours, hierarchy = cv2.findContours(padded, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    resolution = msg.info.resolution
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y

    obstacles = []
    for i, contour in enumerate(contours):
        if hierarchy[0][i][3] != -1:
            continue  # 부모가 있는 컨투어(구멍) — 최상위 외곽선만 취급
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        points = []
        for pt in approx.reshape(-1, 2):
            col = int(pt[0]) - 1  # 패딩으로 밀린 만큼 되돌림
            row = int(pt[1]) - 1
            points.append({
                'x': origin_x + (col + 0.5) * resolution,
                'y': origin_y + (row + 0.5) * resolution,
            })
        obstacles.append(points)
    return obstacles


def _compute_map_boundary(msg: OccupancyGrid) -> list:
    """OccupancyGrid -> 그리드 전체 경계의 4개 모서리 좌표
    ([{'x':..,'y':..}, ...]) — `map_data`의 `map` 필드용.

    점유 여부와 무관하게 `width`/`height`/`resolution`/`origin`만으로 계산하는
    순수 기하 정보라, `occupied_threshold`나 셀 값 자체는 필요 없다.
    origin이 무회전이라고 가정(다른 OccupancyGrid 발행자와 동일한 전제).
    """
    width = msg.info.width
    height = msg.info.height
    if width == 0 or height == 0:
        return []

    resolution = msg.info.resolution
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    far_x = origin_x + width * resolution
    far_y = origin_y + height * resolution

    return [
        {'x': origin_x, 'y': origin_y},
        {'x': far_x, 'y': origin_y},
        {'x': far_x, 'y': far_y},
        {'x': origin_x, 'y': far_y},
    ]


def _detect_local_ip():
    """Best-effort outbound LAN IP, for when `host` is 0.0.0.0 (bind-all)
    and thus not itself a usable address to advertise over mDNS."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))  # no packet actually sent, just a route lookup
        return s.getsockname()[0]
    except OSError:
        return socket.gethostbyname(socket.gethostname())
    finally:
        s.close()


class AppWebSocketBridge(Node):
    """앱↔ROS 양방향 중계 노드.

    - WebSocket 서버는 asyncio 이벤트 루프를 별도 스레드에서 돌린다
      (`rclpy.spin()`이 메인 스레드를 쓰므로, HTTPServer 기반 구노드들이
      쓰던 `threading.Thread(serve_forever)` 패턴과 동일).
    - rclpy 콜백(구독)과 asyncio 코루틴(웹소켓 송수신)이 서로 다른
      스레드에서 돌기 때문에, 콜백 스레드에서 asyncio 쪽으로 넘어갈 때는
      항상 `asyncio.run_coroutine_threadsafe(...)`로 안전하게 핸드오프한다.
    """

    def __init__(self):
        super().__init__('app_websocket_bridge')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8887)
        self.declare_parameter('heartbeat_period_sec', 0.5)
        self.declare_parameter('heartbeat_timeout_sec', 1.5)
        self.declare_parameter('mdns_enabled', True)
        self.declare_parameter('mdns_instance_name', 'jangauto')
        self.declare_parameter('app_status_publish_period_sec', 0.2)
        self.declare_parameter('map_data_retry_timeout_sec', 1.0)
        self.declare_parameter('map_data_max_retries', 3)
        self.declare_parameter('map_data_retry_check_period_sec', 0.2)
        self.declare_parameter('map_data_keepalive_period_sec', 5.0)
        self.declare_parameter('map_occupied_threshold', 99)
        self.declare_parameter('coverage_path_retry_timeout_sec', 1.0)
        self.declare_parameter('coverage_path_max_retries', 3)
        self.declare_parameter('coverage_path_retry_check_period_sec', 0.2)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.heartbeat_period_sec = self.get_parameter('heartbeat_period_sec').get_parameter_value().double_value
        self.heartbeat_timeout_sec = self.get_parameter('heartbeat_timeout_sec').get_parameter_value().double_value
        self.mdns_enabled = self.get_parameter('mdns_enabled').get_parameter_value().bool_value
        self.mdns_instance_name = self.get_parameter('mdns_instance_name').get_parameter_value().string_value
        self.app_status_publish_period_sec = self.get_parameter(
            'app_status_publish_period_sec').get_parameter_value().double_value
        self.map_data_retry_timeout_sec = self.get_parameter(
            'map_data_retry_timeout_sec').get_parameter_value().double_value
        self.map_data_max_retries = self.get_parameter(
            'map_data_max_retries').get_parameter_value().integer_value
        self.map_data_retry_check_period_sec = self.get_parameter(
            'map_data_retry_check_period_sec').get_parameter_value().double_value
        self.map_data_keepalive_period_sec = self.get_parameter(
            'map_data_keepalive_period_sec').get_parameter_value().double_value
        self.map_occupied_threshold = self.get_parameter(
            'map_occupied_threshold').get_parameter_value().integer_value
        self.coverage_path_retry_timeout_sec = self.get_parameter(
            'coverage_path_retry_timeout_sec').get_parameter_value().double_value
        self.coverage_path_max_retries = self.get_parameter(
            'coverage_path_max_retries').get_parameter_value().integer_value
        self.coverage_path_retry_check_period_sec = self.get_parameter(
            'coverage_path_retry_check_period_sec').get_parameter_value().double_value

        # Publishers
        self.control_state_pub = self.create_publisher(String, '/app/control_state', 10)
        self.command_pub = self.create_publisher(String, '/app/command', 10)
        self.link_alive_pub = self.create_publisher(Bool, '/app/link_alive', 10)

        # 앱용 JSON 조립 재료 캐시 — 각 구독 콜백이 최신값만 갱신하고, 실제 조립·발행은
        # _publish_app_status_tick()이 주기 타이머에서 한 번에 처리한다.
        self._last_robot_status = None
        self._last_global_odom = None
        self._last_local_odom = None
        # 조립된 마지막 JSON 문자열 — 신규 WS 클라이언트가 접속하면 다음 타이머 틱까지
        # 기다리지 않고 이걸 바로 보내준다(ROS TRANSIENT_LOCAL이 WS 계층까지는 안 미치므로
        # 이 캐시가 그 역할을 대신함).
        self._last_app_status_json = None

        # 진단(diagnostic_updater)용 최근 수신 시각 — gps_covariance_filler_simul.py와 동일한
        # time.monotonic() 기반 최근성 판정 패턴. 각 _on_* 콜백에서만 갱신한다.
        self._last_map_msg_monotonic = None
        self._last_robot_status_monotonic = None
        self._last_global_odom_monotonic = None
        self._last_local_odom_monotonic = None

        # map_data(map+obstacles) 신뢰성 전송 상태 — 앱의 MoveRequest 재전송 패턴과
        # 동일하게 msg_id 하나를 pending으로 추적하다가 app_ack를 받으면 해제한다.
        self._last_sent_map_payload = None  # (map 경계, obstacles 컨투어 리스트) 튜플
        self._pending_map_data = None  # {'msg_id','json','retry_count','last_sent_monotonic'}
        self._map_data_last_delivery_failed = False
        # 마지막으로 조립된 map_data JSON 문자열 — 신규 클라이언트 접속 시 즉시 전송,
        # 그리고 내용이 안 바뀌어도 map_data_keepalive_period_sec마다 재방송하는 데
        # 쓴다(둘 다 "그 순간 못 받은 클라이언트는 다시는 지도를 못 받는" 문제 방지).
        self._last_map_data_json = None

        # mission_state_machine.py가 /robot_status를 RELIABLE+TRANSIENT_LOCAL(latched)로
        # 발행하므로 구독 쪽도 durability를 맞춰야 late-join 시 마지막 값을 실제로 받는다
        # (구독 쪽이 기본 VOLATILE이면 QoS는 호환되어 에러는 안 나지만, 늦게 구독해도 과거
        # 값을 재생해주지 않고 그 다음 변화부터만 받게 됨 — 실행 확인됨). uwb_virtual_map_publisher_simul.py
        # 의 /map도 같은 latched 계약(map_server 방식)이라 같은 QoS를 재사용한다.
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Status, ROBOT_STATUS_TOPIC, self._on_robot_status, latched_qos)
        self.create_subscription(Odometry, ODOMETRY_GLOBAL_TOPIC, self._on_odometry_global, 10)
        self.create_subscription(Odometry, ODOMETRY_LOCAL_TOPIC, self._on_odometry_local, 10)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, latched_qos)
        self.create_subscription(
            Bool, CALIBRATION_COMPLETE_TOPIC, self._on_calibration_complete, latched_qos)
        self.create_subscription(Twist, CMD_VEL_MANUAL_TOPIC, self._on_cmd_vel_manual, 10)

        # MoveRequest 처리 상태 — 판단·Nav2 액션 클라이언트 연동·WS 응답까지 이 노드가
        # 전담한다(별도 mission 쪽 노드 없음). calibration_complete는 CAL을 한 번이라도
        # 완료했는지 판단하는 재료.
        self._calibration_complete = False
        self._active_move_goal_handle = None
        self._last_move_msg_id = None
        self._nav2_client = ActionClient(self, NavigateToPose, NAV2_ACTION_NAME)
        # wait_for_server처럼 블로킹되는 Nav2 호출을 WS asyncio 스레드/rclpy spin 스레드와
        # 분리하기 위한 전용 워커풀 — 여기서 블로킹돼도 WS 메시지 처리는 안 막힌다.
        self._move_worker_pool = concurrent.futures.ThreadPoolExecutor(
            max_workers=2, thread_name_prefix='move_goal')

        # ㄹ자 커버리지 경로 생성/선택 처리 상태 — move와 마찬가지로 판단·액션
        # 연동·WS 응답까지 이 노드가 전담한다(mission 쪽엔 계산 액션 서버만 있음).
        self._coverage_path_client = ActionClient(
            self, GenerateCoveragePath, GENERATE_COVERAGE_PATH_ACTION_NAME)
        self._coverage_path_worker_pool = concurrent.futures.ThreadPoolExecutor(
            max_workers=2, thread_name_prefix='coverage_path')
        self.selected_coverage_path_pub = self.create_publisher(
            CoveragePath, SELECTED_COVERAGE_PATH_TOPIC, latched_qos)
        self._coverage_path_goal_active = False
        # 최신 coverage_path_result 하나만 유지(map_data와 동일한 "최신 우선"
        # 원칙) — {'msg_id','json','retry_count','last_sent_monotonic',
        # 'candidate_paths','acked'}. candidate_paths는 select_coverage_path가
        # 왔을 때 다시 계산하지 않고 그대로 재사용할 원본 CoveragePath 메시지
        # 2개. `acked`가 True가 돼도(=app_ack 수신) 이 딕셔너리 자체는 지우지
        # 않는다 — 재전송만 멈추고, 사용자가 나중에 select_coverage_path를
        # 보낼 때까지 후보 데이터는 계속 유효해야 하기 때문(선택은 ack보다
        # 한참 뒤에 올 수 있음). select가 성공해도 지우지 않는다 — 같은
        # generate 결과 안에서 마음이 바뀌면 다른 후보로 재선택할 수 있어야
        # 하기 때문(재선택 시 최신 선택으로 latched 토픽만 갱신됨). 지워지는
        # 시점은 새 generate_coverage_path로 교체될 때뿐.
        self._pending_coverage_path = None
        self._coverage_path_last_delivery_failed = False
        # calibration_complete와 동일하게 "한 번이라도 선택했는가"를 앱 상태
        # tick에 노출 — 이 노드가 곧 selected_coverage_path의 발행자이므로
        # 별도 구독 없이 발행 시점에 바로 갱신한다.
        self._path_selected = False

        # Shared state between the asyncio thread and the ROS timer thread.
        # NOTE: names are prefixed with _ws_ to avoid clashing with rclpy's
        # own Node internals (e.g. Node already has a private `_clients`
        # list for ROS service clients — reusing that name here silently
        # corrupts it and breaks the executor/destroy_node).
        self._ws_lock = threading.Lock()
        self._ws_clients = set()
        self._ws_last_msg_monotonic = None
        self._ws_last_link_alive = None

        # Run the WebSocket server's asyncio loop in a background thread so
        # rclpy.spin() can own the main thread, same pattern the old
        # HTTPServer-based nodes used with threading.Thread(serve_forever).
        self._ws_loop = None
        self._ws_server = None
        self._ws_server_thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self._ws_server_thread.start()

        self.create_timer(self.heartbeat_period_sec, self._heartbeat_check)
        self.create_timer(self.app_status_publish_period_sec, self._publish_app_status_tick)
        self.create_timer(self.map_data_retry_check_period_sec, self._map_data_retry_check)
        self.create_timer(self.map_data_keepalive_period_sec, self._map_data_keepalive_tick)
        self.create_timer(
            self.coverage_path_retry_check_period_sec, self._coverage_path_retry_check)

        self._diag_updater = diagnostic_updater.Updater(self)
        self._diag_updater.setHardwareID('app_websocket_bridge')
        self._diag_updater.add('WebSocket link', self._diagnostics_callback)
        self._diag_updater.add('Map reception', self._map_diag_callback)
        self._diag_updater.add('Robot status reception', self._robot_status_diag_callback)
        self._diag_updater.add('Global odometry reception', self._global_odom_diag_callback)
        self._diag_updater.add('Local odometry reception', self._local_odom_diag_callback)
        self._diag_updater.add('Map data delivery', self._map_data_delivery_diag_callback)
        self._diag_updater.add('Coverage path delivery', self._coverage_path_delivery_diag_callback)

        self.get_logger().info(
            f'[AppWsBridge] Starting WebSocket server on {self.host}:{self.port} '
            f'(heartbeat every {self.heartbeat_period_sec}s, timeout {self.heartbeat_timeout_sec}s)')

        self._zeroconf = None
        self._mdns_service_info = None
        if self.mdns_enabled:
            self._advertise_mdns()

    # ------------------------------------------------------------------ mdns
    def _advertise_mdns(self):
        ip = self.host if self.host != '0.0.0.0' else _detect_local_ip()
        try:
            self._zeroconf = Zeroconf()
            self._mdns_service_info = ServiceInfo(
                type_=MDNS_SERVICE_TYPE,
                name=f'{self.mdns_instance_name}.{MDNS_SERVICE_TYPE}',
                addresses=[socket.inet_aton(ip)],
                port=self.port,
                properties={},
                server=f'{self.mdns_instance_name}.local.',
            )
            self._zeroconf.register_service(self._mdns_service_info)
            self.get_logger().info(
                f'[AppWsBridge] mDNS advertised: {self._mdns_service_info.name} @ {ip}:{self.port}')
        except Exception as e:
            self.get_logger().error(f'[AppWsBridge] Failed to advertise mDNS service: {e}')

    # ------------------------------------------------------------ asyncio
    def _run_event_loop(self):
        """백그라운드 스레드의 진입점. 이 스레드 전용 asyncio 이벤트 루프를
        만들고 `_serve()`가 끝날 때까지(=서버 종료 때까지) 블로킹한다."""
        self._ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._ws_loop)
        try:
            self._ws_loop.run_until_complete(self._serve())
        except Exception as e:
            self.get_logger().error(f'[AppWsBridge] WebSocket server crashed: {e}')

    async def _serve(self):
        """실제 WebSocket 서버를 열고, 서버가 닫힐 때까지 대기한다.
        클라이언트가 붙을 때마다 `_handle_client`가 각각 별도 태스크로 호출된다."""
        self._ws_server = await websockets.serve(self._handle_client, self.host, self.port)
        self.get_logger().info(f'[AppWsBridge] WebSocket server listening on {self.host}:{self.port}')
        await self._ws_server.wait_closed()

    async def _handle_client(self, websocket, path=None):
        """클라이언트 1명당 하나씩 실행되는 연결 수명주기 핸들러.

        - 접속하면 클라이언트 집합에 등록하고, 캐시된 최신 상태/지도가 있으면
          바로 한 번씩 보내준다(늦게 접속해도 현재 상태·지도를 즉시 알 수 있게 —
          지도 쪽은 `map_data_keepalive_period_sec` 주기 재방송을 기다릴
          필요 없이 접속 즉시 받게 하기 위함).
        - 이후 들어오는 메시지마다 `_on_message`로 넘긴다.
        - 어떤 예외가 나든(연결 끊김 포함) 다른 클라이언트에 영향 없이
          이 커넥션만 정리하고 끝낸다.
        """
        peer = websocket.remote_address
        self.get_logger().info(f'[AppWsBridge] Client connected: {peer}')
        with self._ws_lock:
            self._ws_clients.add(websocket)
        if self._last_app_status_json is not None:
            try:
                await websocket.send(self._last_app_status_json)
            except Exception as e:
                self.get_logger().warn(f'[AppWsBridge] Failed to send initial status to {peer}: {e}')
        if self._last_map_data_json is not None:
            try:
                await websocket.send(self._last_map_data_json)
            except Exception as e:
                self.get_logger().warn(f'[AppWsBridge] Failed to send initial map_data to {peer}: {e}')
        try:
            async for message in websocket:
                self._on_message(message, peer)
        except ConnectionClosed as e:
            self.get_logger().warn(f'[AppWsBridge] Client {peer} connection closed: {e}')
        except Exception as e:
            # Never let one client's error take the whole server down.
            self.get_logger().error(f'[AppWsBridge] Error handling client {peer}: {e}')
        finally:
            with self._ws_lock:
                self._ws_clients.discard(websocket)
            self.get_logger().info(f'[AppWsBridge] Client disconnected: {peer}')
            # React to disconnects immediately instead of waiting for the
            # next heartbeat tick.
            self._heartbeat_check()

    # ------------------------------------------------------- message handling
    def _on_message(self, message, peer):
        """앱이 보낸 원시 메시지 하나를 파싱해서 알맞은 ROS 토픽으로
        재발행한다 — 내용을 해석하지 않고 JSON을 그대로 문자열째 전달한다
        (`command` 키가 있으면 `/app/command`, 조작 관련 키가 있으면
        `/app/control_state`)."""
        try:
            data = json.loads(message)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warn(f'[AppWsBridge] Invalid JSON from {peer}: {e}', throttle_duration_sec=5.0)
            return

        if not isinstance(data, dict):
            self.get_logger().warn(
                f'[AppWsBridge] Ignoring non-object JSON from {peer}', throttle_duration_sec=5.0)
            return

        with self._ws_lock:
            self._ws_last_msg_monotonic = time.monotonic()

        if 'command' in data:
            out = String()
            out.data = json.dumps(data)
            self.command_pub.publish(out)
            command = data.get('command')
            if command == 'move':
                self._handle_move_command(data, peer)
            elif command == 'generate_coverage_path':
                self._handle_generate_coverage_path_command(data, peer)
            elif command == 'select_coverage_path':
                self._handle_select_coverage_path_command(data, peer)
        elif data.keys() & CONTROL_STATE_KEYS:
            out = String()
            out.data = json.dumps(data)
            self.control_state_pub.publish(out)
        elif data.get('type') == 'app_ack':
            self._on_app_ack(data.get('msg_id'))
        else:
            self.get_logger().warn(
                f'[AppWsBridge] Unrecognized message shape from {peer}: {list(data.keys())}',
                throttle_duration_sec=5.0)

    # ------------------------------------------------------ app status assembly
    def _on_robot_status(self, msg: Status) -> None:
        """`/robot_status` 구독 콜백 — 최신값 캐싱만, 조립·발행은 타이머가 한다."""
        self._last_robot_status = msg
        self._last_robot_status_monotonic = time.monotonic()

    def _on_odometry_global(self, msg: Odometry) -> None:
        """`/odometry/global`(GPS+IMU 전역 EKF) 구독 콜백 — tag_x/tag_y/tag_ori 소스."""
        self._last_global_odom = msg
        self._last_global_odom_monotonic = time.monotonic()

    def _on_odometry_local(self, msg: Odometry) -> None:
        """`/odom`(IMU 로컬 EKF) 구독 콜백 — tag_vel/tag_yaw_rate 소스."""
        self._last_local_odom = msg
        self._last_local_odom_monotonic = time.monotonic()

    # -------------------------------------------------------------- map_data
    def _on_map(self, msg: OccupancyGrid) -> None:
        """`/global_costmap/costmap` 구독 콜백 — 그리드 경계(`map`)와 점유 셀
        컨투어(`obstacles`)를 계산해서, 직전에 보낸 것과 내용이 다르면(dedup)
        새 `map_data` 메시지로 앱에 전송하고 신뢰성 전송 추적을 시작한다.
        내용이 안 바뀌어도 마지막 JSON은 `_last_map_data_json`에 캐싱해둬서
        신규 접속 즉시 전송/주기 재방송(`_map_data_keepalive_tick`)이 쓴다.
        200x200 그리드라 매 틱(최대 5Hz) 추출해도 비용이 미미해서 캐싱 없이
        콜백에서 바로 계산한다.
        """
        self._last_map_msg_monotonic = time.monotonic()
        boundary = _compute_map_boundary(msg)
        obstacles = _extract_obstacle_contours(msg, self.map_occupied_threshold)

        payload_key = (boundary, obstacles)
        if payload_key == self._last_sent_map_payload:
            return
        self._last_sent_map_payload = payload_key

        msg_id = uuid.uuid4().hex[:8]
        payload = {'type': 'map_data', 'msg_id': msg_id, 'map': boundary, 'obstacles': obstacles}
        text = json.dumps(payload)
        self._last_map_data_json = text

        self._pending_map_data = {
            'msg_id': msg_id,
            'json': text,
            'retry_count': 0,
            'last_sent_monotonic': time.monotonic(),
        }
        self.get_logger().info(
            f'[AppWsBridge] Sending map_data [ID: {msg_id}] with {len(obstacles)} '
            f'obstacle(s), {sum(len(o) for o in obstacles)} point(s)')
        self._broadcast_to_clients(text)

    def _on_app_ack(self, msg_id) -> None:
        """앱이 보낸 `app_ack` 처리 — pending map_data/coverage_path_result 중
        msg_id가 일치하는 쪽의 재전송을 멈춘다. map_data는 ack 즉시 완전히
        치워도 되지만(다시 참조할 일 없음), coverage_path_result는 `acked`만
        표시하고 데이터는 남겨둔다 — 나중에 올 select_coverage_path가 여전히
        이 후보를 참조해야 하기 때문."""
        pending = self._pending_map_data
        if pending is not None and pending['msg_id'] == msg_id:
            self._pending_map_data = None
            self._map_data_last_delivery_failed = False
            self.get_logger().info(f'[AppWsBridge] map_data acked [ID: {msg_id}]')
            return

        pending = self._pending_coverage_path
        if pending is not None and pending['msg_id'] == msg_id:
            pending['acked'] = True
            self.get_logger().info(f'[AppWsBridge] coverage_path_result acked [ID: {msg_id}]')

    def _map_data_retry_check(self) -> None:
        """주기 타이머 — pending map_data가 timeout을 넘겼는데 아직 ack가 안
        왔으면 재전송한다(앱의 SocketManager.enqueueRetry와 동일한 패턴:
        고정 timeout, 최대 횟수 후 포기)."""
        pending = self._pending_map_data
        if pending is None:
            return
        elapsed = time.monotonic() - pending['last_sent_monotonic']
        if elapsed < self.map_data_retry_timeout_sec:
            return

        if pending['retry_count'] >= self.map_data_max_retries:
            self.get_logger().warning(
                f"[AppWsBridge] map_data [ID: {pending['msg_id']}] retry exhausted, giving up")
            self._pending_map_data = None
            self._map_data_last_delivery_failed = True
            return

        pending['retry_count'] += 1
        pending['last_sent_monotonic'] = time.monotonic()
        self.get_logger().info(
            f"[AppWsBridge] Retrying map_data [ID: {pending['msg_id']}] "
            f"({pending['retry_count']}/{self.map_data_max_retries})")
        self._broadcast_to_clients(pending['json'])

    def _map_data_keepalive_tick(self) -> None:
        """저주파수 주기 타이머 — 지도 내용이 한동안 안 바뀌어도 마지막
        map_data를 그대로 재방송한다. 그 순간 접속이 없었거나 메시지를
        놓친 클라이언트가 다음 실제 지도 변경까지 기다리지 않고도 결국
        지도를 받게 하기 위함. msg_id+ack 추적(`_pending_map_data`)은
        건드리지 않는 별개 경로 — 그냥 캐시를 다시 내보낼 뿐이라, 앱이
        예전 msg_id로 중복 app_ack를 보내도 `_on_app_ack`가 조용히 무시한다."""
        if self._last_map_data_json is None:
            return
        self._broadcast_to_clients(self._last_map_data_json)

    # -------------------------------------------------------------- move (MoveRequest)
    def _on_calibration_complete(self, msg: Bool) -> None:
        """`/jangauto_mission/calibration_complete` 구독 콜백 — 최신값만 캐싱."""
        self._calibration_complete = msg.data

    def _on_cmd_vel_manual(self, msg: Twist) -> None:
        """조이스틱 출력 구독 콜백 — 0이 아닌 값이 오면(=조작 시작) 진행 중인 move
        goal을 취소한다. `cmd_vel_arbiter.py`의 우선순위(조이스틱이 항상 이김)와는
        별개로, Nav2가 배경에서 goal을 계속 쫓는 것 자체를 멈추기 위함."""
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            return
        goal_handle = self._active_move_goal_handle
        if goal_handle is not None:
            self.get_logger().info('[AppWsBridge] Joystick engaged — cancelling active move goal')
            goal_handle.cancel_goal_async()
            self._active_move_goal_handle = None

    def _handle_move_command(self, data: dict, _peer) -> None:
        """`/app/command`의 `command=="move"` 처리 — 수락 조건을 판단하고, 거부면
        즉시 응답, 수락이면 워커풀에 Nav2 goal 전송을 위임한다."""
        msg_id = data.get('msg_id')
        if msg_id is not None and msg_id == self._last_move_msg_id:
            # 앱의 자체 재시도로 같은 요청이 반복 도착 — 또 goal을 보내지 않는다.
            return
        self._last_move_msg_id = msg_id

        current_state = self._last_robot_status.current_state if self._last_robot_status else None
        if not self._calibration_complete:
            self._send_move_ack(msg_id, False, 'CAL을 아직 완료하지 않음')
            return
        if current_state not in MOVE_ALLOWED_MODES:
            self._send_move_ack(
                msg_id, False,
                f'현재 {current_state} 상태에서는 이동 명령을 받을 수 없음(KEY/CAL만 가능)')
            return

        x = data.get('x')
        y = data.get('y')
        if self._active_move_goal_handle is not None:
            # 최신 요청이 이전 요청을 대체 — map_data와 동일한 "최신 우선" 원칙.
            self._active_move_goal_handle.cancel_goal_async()
            self._active_move_goal_handle = None
        self._move_worker_pool.submit(self._send_nav2_goal, msg_id, x, y)

    def _send_nav2_goal(self, msg_id, x, y) -> None:
        """워커 스레드에서 실행 — Nav2 액션 서버 대기(블로킹)는 이 스레드에서만
        일어나므로 WS/rclpy spin 스레드는 영향받지 않는다."""
        if not self._nav2_client.wait_for_server(timeout_sec=NAV2_WAIT_FOR_SERVER_TIMEOUT_SEC):
            self._send_move_ack(msg_id, False, 'Nav2 액션 서버 응답 없음')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0  # MoveRequest에 방향 필드가 없어 기본값 사용

        self._nav2_client.send_goal_async(goal).add_done_callback(
            lambda future: self._on_nav2_goal_response(future, msg_id))

    def _on_nav2_goal_response(self, future, msg_id) -> None:
        """Nav2가 goal 수락/거부를 응답한 시점(mission_state_machine.py와 동일한
        콜백 패턴 — 블로킹 `.result()` 안 씀)."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._send_move_ack(msg_id, False, 'Nav2가 목표 지점을 거부함')
            return
        self._active_move_goal_handle = goal_handle
        self._send_move_ack(msg_id, True, '')
        goal_handle.get_result_async().add_done_callback(
            lambda future: self._on_nav2_goal_result(future, msg_id))

    def _on_nav2_goal_result(self, future, msg_id) -> None:
        """goal이 끝남(성공/실패/취소 무관) — 더 이상 진행 중이 아니므로 추적 해제.
        완료/도착 자체를 앱에 알리는 기능은 이번 범위 밖(앱도 기대 UI 없음)."""
        if self._last_move_msg_id == msg_id:
            self._active_move_goal_handle = None

    def _send_move_ack(self, msg_id, accepted: bool, reason: str) -> None:
        self._send_simple_ack('move_ack', msg_id, accepted, reason)

    def _send_simple_ack(self, ack_type: str, msg_id, accepted: bool, reason: str) -> None:
        """`{type, msg_id, accepted, reason}` 형태의 1회성 명령 ack 공통 전송 —
        재전송 없음(이미 열린 소켓 위의 직접 응답이라 유실 위험이 낮고, 유실돼도
        앱이 명령을 다시 보내면 됨). `move_ack`/`generate_coverage_path_ack`/
        `select_coverage_path_ack`가 전부 이 형태를 공유한다."""
        text = json.dumps({
            'type': ack_type,
            'msg_id': msg_id,
            'accepted': accepted,
            'reason': reason,
        })
        self._broadcast_to_clients(text)

    # ------------------------------------------------- coverage path (generate/select)
    def _handle_generate_coverage_path_command(self, data: dict, _peer) -> None:
        """`command=="generate_coverage_path"` 처리 — 상태 게이팅과 파라미터
        파싱만 이 스레드(WS 콜백 스레드)에서 하고, 실제 액션 호출(블로킹 대기
        포함)은 전용 워커풀에 위임한다."""
        msg_id = data.get('msg_id')
        current_state = self._last_robot_status.current_state if self._last_robot_status else None

        if current_state not in COVERAGE_PATH_ALLOWED_MODES:
            self._send_simple_ack(
                'generate_coverage_path_ack', msg_id, False,
                f'현재 {current_state} 상태에서는 경로 생성을 할 수 없음(STOP/KEY/CAL만 가능)')
            return
        if self._coverage_path_goal_active:
            self._send_simple_ack(
                'generate_coverage_path_ack', msg_id, False, '이미 경로 생성이 진행 중')
            return

        try:
            polygon_points = [
                Point32(x=float(p['x']), y=float(p['y']), z=0.0) for p in data['polygon']]
            edge_safety_dist = [float(v) for v in data['edge_safety_dist']]
            robot_radius = float(data['robot_radius'])
            yaw_deg = float(data['yaw_deg'])
            ridge_spacing = float(data['ridge_spacing'])
            headland_length = float(data['headland_length'])
            cell_size = float(data.get('cell_size', 0.0))
        except (KeyError, TypeError, ValueError) as e:
            self._send_simple_ack(
                'generate_coverage_path_ack', msg_id, False, f'잘못된 파라미터: {e}')
            return

        self._coverage_path_goal_active = True
        self._coverage_path_worker_pool.submit(
            self._run_generate_coverage_path, msg_id, polygon_points, edge_safety_dist,
            robot_radius, yaw_deg, ridge_spacing, headland_length, cell_size)

    def _run_generate_coverage_path(self, msg_id, polygon_points, edge_safety_dist,
                                     robot_radius, yaw_deg, ridge_spacing,
                                     headland_length, cell_size) -> None:
        """워커 스레드에서 실행 — `generate_coverage_path` 액션은 계산 하나가
        보통 1초 이내로 끝나므로(NavigateToPose처럼 몇 분씩 걸리는 주행이 아님),
        move처럼 "accept"와 "최종 결과"를 분리하지 않고 계산이 끝난 시점에
        `generate_coverage_path_ack` 하나로 accept/reject/실패를 전부 알린다."""
        if not self._coverage_path_client.wait_for_server(
                timeout_sec=NAV2_WAIT_FOR_SERVER_TIMEOUT_SEC):
            self._coverage_path_goal_active = False
            self._send_simple_ack(
                'generate_coverage_path_ack', msg_id, False, '경로 생성 액션 서버 응답 없음')
            return

        goal = GenerateCoveragePath.Goal()
        goal.polygon.points = polygon_points
        goal.edge_safety_dist = edge_safety_dist
        goal.robot_radius = robot_radius
        goal.yaw_deg = yaw_deg
        goal.ridge_spacing = ridge_spacing
        goal.headland_length = headland_length
        goal.cell_size = cell_size

        done_event = threading.Event()
        outcome = {}

        def _on_goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                outcome['error'] = '경로 생성 액션 서버가 goal을 거부함'
                done_event.set()
                return
            result_future = goal_handle.get_result_async()

            def _on_result(rfuture):
                outcome['result'] = rfuture.result().result
                done_event.set()

            result_future.add_done_callback(_on_result)

        self._coverage_path_client.send_goal_async(goal).add_done_callback(_on_goal_response)
        # 워커 스레드 전용 블로킹 — WS/rclpy spin 스레드는 영향받지 않는다(move와 동일 패턴).
        done_event.wait()

        self._coverage_path_goal_active = False

        if 'error' in outcome:
            self._send_simple_ack('generate_coverage_path_ack', msg_id, False, outcome['error'])
            return
        result = outcome['result']
        if not result.success:
            self._send_simple_ack('generate_coverage_path_ack', msg_id, False, result.message)
            return

        self._send_simple_ack('generate_coverage_path_ack', msg_id, True, '')
        self._broadcast_coverage_path_result(list(result.candidate_paths))

    def _broadcast_coverage_path_result(self, candidate_paths: list) -> None:
        """계산된 후보 2개를 `map_data`와 동일한 msg_id+재전송+`app_ack` 패턴으로
        전송한다. `map_data`와 달리 재연결 keepalive는 두지 않는다 — 이건
        "항상 최신이어야 하는 지도 상태"가 아니라 "1회성 제안"이라, 나중에
        재접속한 클라이언트에 오래된 제안을 다시 들이미는 게 오히려 혼란을 줌."""
        result_msg_id = uuid.uuid4().hex[:8]
        payload = {
            'type': 'coverage_path_result',
            'msg_id': result_msg_id,
            'paths': [self._coverage_path_to_json(p) for p in candidate_paths],
        }
        text = json.dumps(payload)

        self._pending_coverage_path = {
            'msg_id': result_msg_id,
            'json': text,
            'retry_count': 0,
            'last_sent_monotonic': time.monotonic(),
            'candidate_paths': candidate_paths,  # select_coverage_path에서 그대로 재사용
            'acked': False,
        }
        self.get_logger().info(f'[AppWsBridge] Sending coverage_path_result [ID: {result_msg_id}]')
        self._broadcast_to_clients(text)

    @staticmethod
    def _coverage_path_to_json(path: CoveragePath) -> dict:
        return {
            'first_row_side': path.first_row_side,
            'rect_length': path.rect_length,
            'rect_width': path.rect_width,
            'work_len': path.work_len,
            'n_ridges': path.n_ridges,
            'start_headland_corners': [
                {'x': c.x, 'y': c.y} for c in path.start_headland_corners
            ],
            'far_headland_corners': [
                {'x': c.x, 'y': c.y} for c in path.far_headland_corners
            ],
            'waypoints': [
                {
                    'x': wp.x, 'y': wp.y, 'yaw': wp.yaw, 'turn_angle': wp.turn_angle,
                    'dist_to_next': wp.dist_to_next, 'kind': wp.kind,
                    'row_index': wp.row_index,
                }
                for wp in path.waypoints
            ],
        }

    def _coverage_path_retry_check(self) -> None:
        """주기 타이머 — `_map_data_retry_check`와 동일한 패턴(고정 timeout,
        최대 횟수 후 포기), keepalive만 없음. 이미 `acked`된 뒤에는 select
        대기 중인 것뿐이므로 재전송하지 않고 그냥 넘어간다(데이터는 유지)."""
        pending = self._pending_coverage_path
        if pending is None or pending['acked']:
            return
        elapsed = time.monotonic() - pending['last_sent_monotonic']
        if elapsed < self.coverage_path_retry_timeout_sec:
            return

        if pending['retry_count'] >= self.coverage_path_max_retries:
            self.get_logger().warning(
                f"[AppWsBridge] coverage_path_result [ID: {pending['msg_id']}] "
                f"retry exhausted, giving up")
            self._pending_coverage_path = None
            self._coverage_path_last_delivery_failed = True
            return

        pending['retry_count'] += 1
        pending['last_sent_monotonic'] = time.monotonic()
        self.get_logger().info(
            f"[AppWsBridge] Retrying coverage_path_result [ID: {pending['msg_id']}] "
            f"({pending['retry_count']}/{self.coverage_path_max_retries})")
        self._broadcast_to_clients(pending['json'])

    def _handle_select_coverage_path_command(self, data: dict, _peer) -> None:
        """`command=="select_coverage_path"` 처리 — `ref_msg_id`로 직전
        `coverage_path_result`를 참조하고 `path_index`(0/1)로 후보를 골라
        `selected_coverage_path`(latched)에 publish한다. `msg_id`는 다른 명령들과
        동일하게 앱이 매번 새로 발급하는 이 요청 자체의 추적/ack용 ID이고,
        "어떤 결과를 선택하는지"는 별도 `ref_msg_id` 필드로 명시한다(하나의
        `msg_id` 필드가 명령마다 다른 의미를 갖는 걸 피하기 위함). 이 select
        요청 자체가 해당 결과를 받았다는 증거이므로 별도 `app_ack` 없이도
        재전송을 멈춘다.

        같은 `ref_msg_id`로 여러 번 호출 가능 — 재선택할 때마다 그 값으로
        `selected_coverage_path`를 덮어써서 갱신한다(마음이 바뀌어 좌/우측
        후보를 바꿔 고르는 경우 지원). `_pending_coverage_path`는 여기서
        지우지 않는다 — 새 `generate_coverage_path`가 올 때만 교체된다."""
        msg_id = data.get('msg_id')
        ref_msg_id = data.get('ref_msg_id')
        path_index = data.get('path_index')
        current_state = self._last_robot_status.current_state if self._last_robot_status else None

        pending = self._pending_coverage_path
        if pending is None or pending['msg_id'] != ref_msg_id:
            self._send_simple_ack(
                'select_coverage_path_ack', msg_id, False,
                '참조한 경로 결과를 찾을 수 없음(만료되었거나 잘못된 ref_msg_id)')
            return
        if current_state not in COVERAGE_PATH_ALLOWED_MODES:
            self._send_simple_ack(
                'select_coverage_path_ack', msg_id, False,
                f'현재 {current_state} 상태에서는 경로 선택을 할 수 없음(STOP/KEY/CAL만 가능)')
            return
        if path_index not in (0, 1):
            self._send_simple_ack(
                'select_coverage_path_ack', msg_id, False, f'잘못된 path_index: {path_index}')
            return

        selected = pending['candidate_paths'][path_index]
        self.selected_coverage_path_pub.publish(selected)
        self._path_selected = True
        # _pending_coverage_path는 지우지 않는다 — 같은 결과 안에서 재선택
        # 가능하게 유지(새 generate_coverage_path로만 교체됨).

        self.get_logger().info(
            f'[AppWsBridge] Coverage path selected: index={path_index} [ref={ref_msg_id}]')
        self._send_simple_ack('select_coverage_path_ack', msg_id, True, '')

    def _publish_app_status_tick(self) -> None:
        """주기 타이머 — 캐시된 `/robot_status`+`/odometry/global`+`/odom`을 모아
        앱용 JSON을 조립해 WS로 브로드캐스트한다(ROS 토픽 재발행 없음). ack가 없는
        지금, 이게 앱에 상태를 알리는 유일한 채널이자 로봇→앱 하트비트를 겸한다.
        """
        status = self._last_robot_status
        if status is None:
            return  # /robot_status를 아직 한 번도 못 받음(부팅 직후) — 보낼 게 없음

        payload = {
            'current_state': status.current_state,
            'in_error': status.in_error,
            'error_reason': status.error_reason,
            'calibration_complete': self._calibration_complete,
            'path_selected': self._path_selected,
        }

        odom = self._last_global_odom
        if odom is not None:
            payload['tag_x'] = odom.pose.pose.position.x
            payload['tag_y'] = odom.pose.pose.position.y
            payload['tag_ori'] = _quaternion_to_yaw(odom.pose.pose.orientation)

        local_odom = self._last_local_odom
        if local_odom is not None:
            payload['tag_vel'] = local_odom.twist.twist.linear.x
            payload['tag_yaw_rate'] = local_odom.twist.twist.angular.z

        text = json.dumps(payload)
        self._last_app_status_json = text
        self._broadcast_to_clients(text)

    def _broadcast_to_clients(self, text: str) -> None:
        """rclpy 콜백 스레드에서 asyncio 이벤트 루프로 안전하게 넘겨서,
        지금 연결된 모든 웹소켓 클라이언트에 텍스트를 전송한다.
        `_stop_ws_server()`가 쓰는 것과 같은 스레드 간 핸드오프 패턴이다."""
        if self._ws_loop is None or not self._ws_loop.is_running():
            return
        asyncio.run_coroutine_threadsafe(self._async_broadcast(text), self._ws_loop)

    async def _async_broadcast(self, text: str) -> None:
        with self._ws_lock:
            clients = list(self._ws_clients)
        if not clients:
            return
        await asyncio.gather(*(c.send(text) for c in clients), return_exceptions=True)

    # -------------------------------------------------------------- heartbeat
    def _heartbeat_check(self):
        """주기 타이머(및 클라이언트 접속/해제 시 즉시)로 호출 — "지금 연결된
        앱이 있고, 최근 heartbeat_timeout_sec 안에 메시지를 받았는가"를
        판단해 `/app/link_alive`에 발행한다. 값이 바뀔 때만 로그를 남긴다."""
        now = time.monotonic()
        with self._ws_lock:
            has_clients = bool(self._ws_clients)
            last = self._ws_last_msg_monotonic
        alive = has_clients and last is not None and (now - last) <= self.heartbeat_timeout_sec

        changed = alive != self._ws_last_link_alive
        self._ws_last_link_alive = alive
        msg = Bool()
        msg.data = alive
        try:
            self.link_alive_pub.publish(msg)
        except Exception:
            # Can race against rclpy's own SIGINT-triggered context teardown
            # when a client disconnects mid-shutdown — safe to drop, we're
            # already on our way out.
            return
        if changed:
            self.get_logger().info(f'[AppWsBridge] link_alive -> {alive}')

    # -------------------------------------------------------------- diagnostics
    def _diagnostics_callback(self, stat):
        if self._ws_server is None:
            stat.summary(DiagnosticStatus.ERROR, 'WebSocket server is not listening')
        elif not self._ws_last_link_alive:
            stat.summary(DiagnosticStatus.WARN, 'No app currently connected')
        else:
            stat.summary(DiagnosticStatus.OK, 'App connected')
        stat.add('mdns_registered', str(self._mdns_service_info is not None))
        return stat

    def _staleness_diag(self, stat, last_monotonic, timeout_sec, label):
        """gps_covariance_filler_simul.py와 동일한 최근성 판정 헬퍼 —
        없음=ERROR / timeout_sec 초과=WARN / 정상=OK."""
        if last_monotonic is None:
            stat.summary(DiagnosticStatus.ERROR, f'No {label} received yet')
        elif (time.monotonic() - last_monotonic) > timeout_sec:
            stat.summary(DiagnosticStatus.WARN, f'{label} is stale')
        else:
            stat.summary(DiagnosticStatus.OK, f'Receiving {label}')
        return stat

    def _map_diag_callback(self, stat):
        return self._staleness_diag(
            stat, self._last_map_msg_monotonic, MAP_STALE_TIMEOUT_SEC, '/global_costmap/costmap')

    def _robot_status_diag_callback(self, stat):
        return self._staleness_diag(
            stat, self._last_robot_status_monotonic, ROBOT_STATUS_STALE_TIMEOUT_SEC, '/robot_status')

    def _global_odom_diag_callback(self, stat):
        return self._staleness_diag(
            stat, self._last_global_odom_monotonic, ODOM_STALE_TIMEOUT_SEC, '/odometry/global')

    def _local_odom_diag_callback(self, stat):
        return self._staleness_diag(
            stat, self._last_local_odom_monotonic, ODOM_STALE_TIMEOUT_SEC, '/odom')

    def _map_data_delivery_diag_callback(self, stat):
        if self._map_data_last_delivery_failed:
            stat.summary(DiagnosticStatus.ERROR, 'Last map_data delivery exhausted retries')
        elif self._pending_map_data is not None:
            stat.summary(
                DiagnosticStatus.WARN,
                f"Retrying map_data [ID: {self._pending_map_data['msg_id']}] "
                f"({self._pending_map_data['retry_count']}/{self.map_data_max_retries})")
        else:
            stat.summary(DiagnosticStatus.OK, 'No pending map_data')
        return stat

    def _coverage_path_delivery_diag_callback(self, stat):
        pending = self._pending_coverage_path
        if self._coverage_path_last_delivery_failed:
            stat.summary(DiagnosticStatus.ERROR, 'Last coverage_path_result delivery exhausted retries')
        elif pending is not None and pending['acked']:
            stat.summary(
                DiagnosticStatus.OK,
                f"Awaiting select_coverage_path [ID: {pending['msg_id']}]")
        elif pending is not None:
            stat.summary(
                DiagnosticStatus.WARN,
                f"Retrying coverage_path_result [ID: {pending['msg_id']}] "
                f"({pending['retry_count']}/{self.coverage_path_max_retries})")
        else:
            stat.summary(DiagnosticStatus.OK, 'No pending coverage_path_result')
        return stat

    # ---------------------------------------------------------------- cleanup
    def destroy_node(self):
        """노드 종료 시 mDNS 등록 해제 + 웹소켓 서버 정리까지 마친 뒤
        상위 `Node.destroy_node()`를 호출한다."""
        self.get_logger().info('[AppWsBridge] Shutting down App WebSocket Bridge...')
        self._move_worker_pool.shutdown(wait=False)
        self._coverage_path_worker_pool.shutdown(wait=False)
        if self._zeroconf is not None:
            try:
                if self._mdns_service_info is not None:
                    self._zeroconf.unregister_service(self._mdns_service_info)
                self._zeroconf.close()
            except Exception as e:
                self.get_logger().error(f'[AppWsBridge] Error tearing down mDNS: {e}')
        self._stop_ws_server()
        super().destroy_node()

    def _stop_ws_server(self):
        """연결된 클라이언트를 먼저 정상 종료(close)시킨 뒤 서버 소켓을
        닫고, 그제서야 asyncio 루프 자체를 멈춘다 — 순서를 바꾸면
        `wait_closed()`가 영원히 안 끝나고 `_run_event_loop`가 가짜 crash
        로그를 남긴다(아래 주석 참고)."""
        if self._ws_loop is None or not self._ws_loop.is_running():
            return

        async def _shutdown():
            # Close any still-open client connections first, while the loop
            # is fully alive, so their close handshakes (and the resulting
            # _handle_client finally-block heartbeat update) can complete
            # cleanly instead of racing the loop being stopped underneath them.
            with self._ws_lock:
                clients = list(self._ws_clients)
            if clients:
                await asyncio.gather(
                    *(c.close(code=1001, reason='Server shutting down') for c in clients),
                    return_exceptions=True,
                )
            if self._ws_server is not None:
                self._ws_server.close()
                await self._ws_server.wait_closed()

        # Let the close() coroutine actually finish inside its own loop
        # before stopping that loop — stopping it first leaves wait_closed()
        # forever pending and _run_event_loop logs a spurious "crashed" error.
        future = asyncio.run_coroutine_threadsafe(_shutdown(), self._ws_loop)
        try:
            future.result(timeout=2.0)
        except Exception as e:
            self.get_logger().warn(f'[AppWsBridge] WebSocket server did not close cleanly: {e}')

        self._ws_loop.call_soon_threadsafe(self._ws_loop.stop)
        self._ws_server_thread.join(timeout=2.0)


def main(args=None):
    """노드 진입점 — `rclpy.spin()`으로 상주하며 콜백을 처리하다가
    Ctrl+C(SIGINT) 시 정리하고 종료한다."""
    rclpy.init(args=args)
    node = AppWebSocketBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
