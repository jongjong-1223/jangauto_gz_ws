#!/usr/bin/env python3
"""App WiFi Transmitter: Send UWB/EKF data + topic/node health to mobile app

Endpoints (GET):
  - /to_app        -> UWB anchors + tag position + current global_path (for the map figure)
  - /topic_status  -> liveness of monitored topics (is data flowing?)
  - /node_status   -> which required nodes are alive / missing
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


# Topics to monitor (name -> ROS message type). "Core" set + /local_path_map.
MONITORED_TOPICS = [
    ('/robot_state',          String),
    ('/cmd_vel',              Twist),
    ('/imu_data',             Imu),
    ('/uwb_raw_data',         String),
    ('/odometry/ekf_single',  Odometry),
    ('/local_path_map',       Path),
]

# Required nodes during REAL driving (real_launch.py, use_fake_odom:=false).
# These are the RUNTIME names (after launch `name=` remapping).
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

# A topic is considered "OK" if a message arrived within this many seconds.
TOPIC_FRESH_SEC = 2.0


class AppDataHandler(BaseHTTPRequestHandler):
    """HTTP request handler"""

    def __init__(self, *args, ros_node=None, **kwargs):
        self.ros_node = ros_node
        super().__init__(*args, **kwargs)

    def _send_json(self, json_str):
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
        pass


class AppWiFiTransmitter(Node):
    def __init__(self):
        super().__init__('app_wifi_transmitter')

        # ---- Existing subscriptions (UWB + EKF, used by the map figure) ----
        self.uwb_subscription = self.create_subscription(
            String, '/uwb_raw_data', self.uwb_data_callback, 1)
        self.ekf_subscription = self.create_subscription(
            Odometry, '/odometry/ekf_single', self.ekf_data_callback, 1)

        # ---- New: subscribe to global path so the figure can draw it ----
        self.global_path_points = []  # list of [x, y]
        self.create_subscription(Path, '/global_path', self.global_path_callback, 10)

        # ---- New: monitoring subscriptions (topic liveness) ----
        self.topic_last_time = {}   # name -> monotonic time of last msg
        self.topic_last_info = {}   # name -> short value summary string
        self._mon_lock = threading.Lock()
        for name, msg_type in MONITORED_TOPICS:
            self.topic_last_time[name] = None
            self.topic_last_info[name] = ''
            # default QoS (RELIABLE, depth 10) matches all monitored publishers
            self.create_subscription(
                msg_type, name,
                self._make_monitor_cb(name, msg_type), 10)

        # HTTP server settings
        self.declare_parameter('port', 8888)
        self.declare_parameter('host', '0.0.0.0')
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value

        # Data storage for the figure
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
        def cb(msg):
            with self._mon_lock:
                self.topic_last_time[name] = time.monotonic()
                self.topic_last_info[name] = self._summarize(name, msg_type, msg)
        return cb

    def _summarize(self, name, msg_type, msg):
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
        yaw_rad = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return math.degrees(yaw_rad)

    def global_path_callback(self, msg):
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]
        with self.data_lock:
            self.global_path_points = pts

    def ekf_data_callback(self, msg):
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