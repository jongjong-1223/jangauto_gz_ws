#!/usr/bin/env python3
"""앱 WiFi 송신기: UWB/EKF 위치 데이터 + 토픽/노드 생존 상태를 모바일 앱에 전송.

## 역할
- (참고용 미사용 코드) 예전 HTTP 기반 앱 프로토콜에서 쓰던 노드다. 현재는
  `app_websocket_bridge.py`(웹소켓 기반 신규 프로토콜)가 이 역할을 대신하며,
  이 파일은 빌드/실행 대상이 아니다.
- 자체 HTTP 서버(`http.server`)를 별도 스레드로 띄워 앱의 GET 요청에 응답한다.
  ROS2 구독 콜백들은 최신값을 내부 상태에 저장만 하고, 실제 HTTP 응답
  생성은 요청이 들어올 때(HTTP 스레드에서) 그 상태를 읽어 조립한다.
- GET 엔드포인트 3종:
  - `/to_app`: UWB 앵커 4개 좌표 + 태그(로봇) 위치/속도/방향 + 현재
    전역 경로 — 앱 지도 화면을 그리는 데 필요한 데이터 전부.
  - `/topic_status`: `MONITORED_TOPICS`에 정의된 핵심 토픽들이 최근
    `TOPIC_FRESH_SEC`초 안에 메시지를 받았는지(살아있는지) 보고.
  - `/node_status`: `REQUIRED_NODES`에 정의된 실주행 필수 노드들이
    현재 ROS 그래프에 떠 있는지 보고.

## 클래스 구성
- `AppDataHandler`: `BaseHTTPRequestHandler`를 상속한 HTTP 요청 처리기.
  요청마다 새 인스턴스가 생성되며, `ros_node` 참조를 통해 실제 JSON
  조립을 위임한다.
- `AppWiFiTransmitter`: ROS2 노드 본체. UWB/EKF/전역경로 구독, 모니터링
  대상 토픽 구독, HTTP 서버 기동/종료, 3개 엔드포인트용 JSON 조립을 담당한다.

## main()의 동작 순서
1. rclpy 초기화
2. `AppWiFiTransmitter` 노드 생성 — UWB/EKF/global_path 구독, 모니터링용
   구독 전체 등록, HTTP 서버를 별도 스레드로 기동(포트 충돌 시 자동으로
   다음 포트 재시도)까지 이 시점에 완료
3. `rclpy.spin()` — ROS2 콜백 처리 루프(블로킹). 콜백은 최신값을 내부
   딕셔너리/캐시에 저장하기만 하고, 실제 JSON 응답은 별도 HTTP 스레드가
   요청을 받을 때 조립한다.
4. Ctrl+C 시 HTTP 서버까지 함께 정리 후 종료
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import json
import threading
import math
import time
from http.server import HTTPServer, BaseHTTPRequestHandler


# 생존 여부를 모니터링할 토픽 목록 (토픽명 -> ROS 메시지 타입).
# "핵심" 토픽 집합 + /local_path_map.
MONITORED_TOPICS = [
    ('/robot_state',          String),
    ('/cmd_vel',              Twist),
    ('/imu_data',             Imu),
    ('/uwb_raw_data',         String),
    ('/odometry/ekf_single',  Odometry),
    ('/local_path_map',       Path),
]

# 실주행 중(real_launch.py, use_fake_odom:=false) 반드시 떠 있어야 하는 노드 목록.
# 여기 적힌 이름은 launch의 `name=` 리매핑을 거친 실행 시점 이름이다.
REQUIRED_NODES = [
    'twist_mux',
    'ekf_local',
    'ekf_global',
    'state_manager',
    'state_machine_executor',
    'app_bridge',
    'pure_pursuit_controller',
    'app_wifi_receiver_node',
    'app_wifi_transmitter_node',
    'motor_cmd_vel_trx_node',
    'wt901c_imu_node',
    'imu_offset_node',
    'uwb_publisher_node',
    'uwb_filter_and_cov_node',
    'centerline_depth_node',
    'local_path_node',
    'tf_imu_to_base',
    'odom_to_chassis_static',
]

# 이 시간(초) 안에 메시지가 도착했으면 해당 토픽을 "정상(OK)"으로 간주.
TOPIC_FRESH_SEC = 2.0


class AppDataHandler(BaseHTTPRequestHandler):
    """HTTP 요청 처리기. 요청 1건당 인스턴스 1개가 생성되므로 상태를 들고
    있지 않고, `ros_node`(AppWiFiTransmitter)가 미리 구성해둔 JSON 문자열을
    그대로 응답으로 내보내는 얇은 어댑터다."""

    def __init__(self, *args, ros_node=None, **kwargs):
        self.ros_node = ros_node
        super().__init__(*args, **kwargs)

    def _send_json(self, json_str):
        """이미 문자열로 직렬화된 JSON을 그대로 응답. CORS 허용 헤더 포함."""
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json_str.encode('utf-8'))

    def do_GET(self):
        try:
            if self.path == '/to_app':
                self._send_json(self.ros_node.get_uwb_json_data() if self.ros_node
                                else json.dumps({"error": "no node"}))
            elif self.path == '/topic_status':
                self._send_json(self.ros_node.get_topic_status_json() if self.ros_node
                                else json.dumps({"error": "no node"}))
            elif self.path == '/node_status':
                self._send_json(self.ros_node.get_node_status_json() if self.ros_node
                                else json.dumps({"error": "no node"}))
            else:
                self.send_error(404, "Not Found")
        except Exception as e:
            self.send_error(500, f"Server error: {e}")

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def log_message(self, format, *args):
        # 기본 stderr 로그 억제 — 폴링 주기가 잦아 로그가 넘치는 걸 방지
        pass


class AppWiFiTransmitter(Node):
    def __init__(self):
        super().__init__('app_wifi_transmitter')

        # ---- 기존 구독: UWB + EKF (지도 화면 데이터용) ----
        self.uwb_subscription = self.create_subscription(
            String, '/uwb_raw_data', self.uwb_data_callback, 1)
        self.ekf_subscription = self.create_subscription(
            Odometry, '/odometry/ekf_single', self.ekf_data_callback, 1)

        # ---- 신규: 지도에 경로를 그릴 수 있도록 전역 경로 구독 ----
        self.global_path_points = []  # [x, y] 좌표 리스트
        self.create_subscription(Path, '/global_path', self.global_path_callback, 10)

        # ---- 신규: 모니터링용 구독 (토픽 생존 여부 확인) ----
        self.topic_last_time = {}   # 토픽명 -> 마지막 수신 시각(monotonic)
        self.topic_last_info = {}   # 토픽명 -> 마지막 값 요약 문자열
        self._mon_lock = threading.Lock()
        for name, msg_type in MONITORED_TOPICS:
            self.topic_last_time[name] = None
            self.topic_last_info[name] = ''
            # 기본 QoS(RELIABLE, depth 10) 사용 — 모니터링 대상 퍼블리셔들과 동일하게 맞춤
            self.create_subscription(
                msg_type, name,
                self._make_monitor_cb(name, msg_type), 10)

        # HTTP 서버 설정
        self.declare_parameter('port', 8888)
        self.declare_parameter('host', '0.0.0.0')
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value

        # 지도 화면용 데이터 저장소
        self.latest_uwb_data = None
        self.has_ekf_data = False
        self.ekf_callback_count = 0
        self.uwb_json_data = {
            "anchors": [
                {"id": "A1", "x": 0.00, "y": 0.00},
                {"id": "A2", "x": 0.00, "y": 0.00},
                {"id": "A3", "x": 0.00, "y": 0.00},
                {"id": "A4", "x": 0.00, "y": 0.00}
            ],
            "tag": {"x": 0.00, "y": 0.00},
            "stop_flag": 0,
            "err_msg": [],
            "tag_vel": 0.00,
            "tag_ori": 0.00,
            "global_path": []
        }
        self.data_lock = threading.Lock()
        self.request_count = 0
        self.data_count = 0

        self.start_http_server()
        self.get_logger().info(f'[WIFI_TX] App WiFi Transmitter started on {self.host}:{self.port}')
        self.get_logger().info('[WIFI_TX] GET endpoints: /to_app  /topic_status  /node_status')

    # ----------------------------------------------------------- monitoring
    def _make_monitor_cb(self, name, msg_type):
        """토픽별 모니터링 콜백을 생성하는 팩토리. 클로저로 토픽명/타입을
        캡처해, 수신 시각과 값 요약을 `topic_last_time`/`topic_last_info`에 기록한다."""
        def cb(msg):
            with self._mon_lock:
                self.topic_last_time[name] = time.monotonic()
                self.topic_last_info[name] = self._summarize(name, msg_type, msg)
        return cb

    def _summarize(self, name, msg_type, msg):
        """모니터링 대상 메시지를 타입별로 사람이 읽기 쉬운 한 줄 요약으로 변환.
        `/topic_status` 응답의 info 필드에 그대로 노출된다."""
        try:
            if msg_type is String:
                s = str(msg.data)
                return s if len(s) <= 40 else s[:37] + '...'
            if msg_type is Twist:
                return f"v={msg.linear.x:.2f} w={msg.angular.z:.2f}"
            if msg_type is Imu:
                o = msg.orientation
                return f"q=({o.x:.2f},{o.y:.2f},{o.z:.2f},{o.w:.2f})"
            if msg_type is Odometry:
                p = msg.pose.pose.position
                return f"x={p.x:.2f} y={p.y:.2f}"
            if msg_type is Path:
                return f"{len(msg.poses)} poses"
        except Exception:
            return ''
        return ''

    def get_topic_status_json(self):
        """`/topic_status` 응답 조립. 각 토픽의 마지막 수신 이후 경과 시간이
        `TOPIC_FRESH_SEC` 이하면 정상, 넘으면 비정상, 한 번도 안 왔으면
        age=-1로 표시한다."""
        now = time.monotonic()
        topics = []
        with self._mon_lock:
            for name, _ in MONITORED_TOPICS:
                last = self.topic_last_time[name]
                if last is None:
                    topics.append({"name": name, "ok": False, "age": -1.0,
                                   "info": "no data yet"})
                else:
                    age = now - last
                    topics.append({"name": name, "ok": age <= TOPIC_FRESH_SEC,
                                   "age": round(age, 2),
                                   "info": self.topic_last_info[name]})
        return json.dumps({"fresh_sec": TOPIC_FRESH_SEC, "topics": topics})

    def get_node_status_json(self):
        """`/node_status` 응답 조립. ROS 그래프에서 현재 살아있는 노드 이름
        집합을 조회해 `REQUIRED_NODES`와 비교하고, 누락된 필수 노드(missing)와
        목록에 없는 여분 노드(extra)를 함께 보고한다."""
        try:
            running = {n for (n, _ns) in self.get_node_names_and_namespaces()}
        except Exception as e:
            return json.dumps({"error": f"node discovery failed: {e}", "nodes": []})
        nodes = [{"name": n, "alive": (n in running)} for n in REQUIRED_NODES]
        missing = [n for n in REQUIRED_NODES if n not in running]
        extra = sorted(running - set(REQUIRED_NODES))
        return json.dumps({"nodes": nodes, "missing": missing, "extra": extra})

    # ---------------------------------------------------------- HTTP server
    def start_http_server(self):
        """HTTP 서버를 데몬 스레드로 기동. 지정 포트가 이미 사용 중이면
        `max_retries`번까지 포트를 하나씩 올려가며 재시도한다(여러 인스턴스가
        동시에 뜨거나 이전 프로세스가 아직 정리 중인 경우 대비)."""
        def handler_factory(*args, **kwargs):
            return AppDataHandler(*args, ros_node=self, **kwargs)
        max_retries = 5
        for retry in range(max_retries):
            try:
                current_port = self.port + retry
                self.server = HTTPServer((self.host, current_port), handler_factory)
                self.server.timeout = 1.0
                self.server_thread = threading.Thread(
                    target=self.serve_forever_with_shutdown, daemon=True)
                self.server_thread.start()
                self.get_logger().info(f'[WIFI_TX] HTTP Server listening on {self.host}:{current_port}')
                self.port = current_port
                break
            except OSError as e:
                if e.errno == 98:
                    self.get_logger().warn(f'[WIFI_TX] Port {current_port} in use, trying {current_port + 1}')
                    continue
                else:
                    self.get_logger().error(f'[WIFI_TX] Failed to start HTTP server: {e}')
                    break
        else:
            self.get_logger().error(f'[WIFI_TX] Could not find available port after {max_retries} attempts')

    def serve_forever_with_shutdown(self):
        try:
            self.server.serve_forever()
        except Exception as e:
            self.get_logger().error(f'[WIFI_TX] HTTP server error: {e}')

    # ------------------------------------------------------ figure data
    def quaternion_to_yaw(self, x, y, z, w):
        """쿼터니언에서 yaw(방향)만 추출해 도(degree) 단위로 변환.
        앱 지도 화면은 2D라 yaw 하나만 있으면 로봇 방향 표시가 충분하다."""
        yaw_rad = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return math.degrees(yaw_rad)

    def global_path_callback(self, msg):
        """전역 경로 갱신. `data_lock`으로 보호해 HTTP 스레드의
        `get_uwb_json_data()` 읽기와 경합하지 않도록 한다."""
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]
        with self.data_lock:
            self.global_path_points = pts

    def ekf_data_callback(self, msg):
        """EKF 추정 위치/속도/방향을 저장. 50콜백에 1번만 갱신하는 이유는
        지도 화면 갱신 빈도가 EKF 발행 빈도만큼 높을 필요가 없어서(부하 절감)."""
        try:
            with self.data_lock:
                self.ekf_callback_count += 1
                self.has_ekf_data = True
                if self.ekf_callback_count % 50 == 0:
                    position_x = msg.pose.pose.position.x
                    position_y = msg.pose.pose.position.y
                    vel_x = msg.twist.twist.linear.x
                    vel_y = msg.twist.twist.linear.y
                    velocity_magnitude = math.sqrt(vel_x ** 2 + vel_y ** 2)
                    orientation = msg.pose.pose.orientation
                    yaw_deg = self.quaternion_to_yaw(
                        orientation.x, orientation.y, orientation.z, orientation.w)
                    self.uwb_json_data["tag"]["x"] = round(position_x, 2)
                    self.uwb_json_data["tag"]["y"] = round(position_y, 2)
                    self.uwb_json_data["tag_vel"] = round(velocity_magnitude, 2)
                    self.uwb_json_data["tag_ori"] = round(yaw_deg, 2)
        except Exception as e:
            self.get_logger().error(f'[WIFI_TX] Error processing EKF data: {e}')

    def uwb_data_callback(self, msg):
        """UWB 앵커 4개의 원시 좌표(콤마 구분 8개 숫자: x1,y1,...,x4,y4)를
        파싱해 저장. 8개 미만이면(데이터 손상/불완전) 무시하고 이전 값 유지."""
        try:
            with self.data_lock:
                self.latest_uwb_data = msg.data
                self.data_count += 1
                data_parts = [float(x.strip()) for x in msg.data.split(',')]
                if len(data_parts) >= 8:
                    self.uwb_json_data["anchors"] = [
                        {"id": "A1", "x": round(data_parts[0], 2), "y": round(data_parts[1], 2)},
                        {"id": "A2", "x": round(data_parts[2], 2), "y": round(data_parts[3], 2)},
                        {"id": "A3", "x": round(data_parts[4], 2), "y": round(data_parts[5], 2)},
                        {"id": "A4", "x": round(data_parts[6], 2), "y": round(data_parts[7], 2)}
                    ]
        except Exception as e:
            self.get_logger().error(f'[WIFI_TX] Error processing UWB data: {e}')

    def get_uwb_json_data(self):
        """`/to_app` 응답 조립. EKF/UWB 데이터가 아직 없으면 stop_flag=1과
        함께 에러 메시지를 채워, 앱이 "아직 위치 추정 전"임을 알 수 있게 한다."""
        with self.data_lock:
            self.request_count += 1
            self.uwb_json_data["err_msg"] = []
            if not self.has_ekf_data:
                self.uwb_json_data["stop_flag"] = 1
                self.uwb_json_data["err_msg"].append("No EKF data available")
            else:
                self.uwb_json_data["stop_flag"] = 0
            if self.latest_uwb_data is None:
                self.uwb_json_data["err_msg"].append("No anchor data available")
            self.uwb_json_data["global_path"] = list(self.global_path_points)
            return json.dumps(self.uwb_json_data)

    # -------------------------------------------------------------- cleanup
    def destroy_node(self):
        self.get_logger().info('[WIFI_TX] Shutting down App WiFi Transmitter...')
        if hasattr(self, 'server'):
            self.server.shutdown()
            self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AppWiFiTransmitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
