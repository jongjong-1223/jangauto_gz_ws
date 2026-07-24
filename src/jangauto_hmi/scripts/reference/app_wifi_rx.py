#!/usr/bin/env python3
"""App WiFi Receiver: Receive JSON data from mobile app and publish to ROS2 topics

Added endpoints (vs. original):
  - POST /poweroff      -> run `sudo poweroff` (needs sudoers NOPASSWD, see SETUP_poweroff_sudoers.md)
  - POST /global_path   -> generate boustrophedon waypoints from app params,
                           publish nav_msgs/Path on /global_path (re-published at 1 Hz)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import json
import threading
import subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler


class AppDataHandler(BaseHTTPRequestHandler):
    """HTTP request handler"""
    def __init__(self, *args, ros_node=None, **kwargs):
        self.ros_node = ros_node
        super().__init__(*args, **kwargs)

    def _read_json_body(self):
        content_length = int(self.headers.get('Content-Length', 0))
        if content_length <= 0:
            return {}
        post_data = self.rfile.read(content_length)
        return json.loads(post_data.decode('utf-8'))

    def _send_json(self, code, payload):
        body = json.dumps(payload).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    # Handle HTTP POST requests from the mobile app.
    def do_POST(self):
        # ---- Existing: command bits ----
        if self.path == '/to_rasp':
            try:
                json_data = self._read_json_body()
                if self.ros_node:
                    self.ros_node.process_app_data(json_data)
                self._send_json(200, {"status": "ok"})
            except json.JSONDecodeError as e:
                self.send_error(400, f"Invalid JSON: {e}")
            except Exception as e:
                self.send_error(500, f"Server error: {e}")

        # ---- New: power off the Raspberry Pi ----
        elif self.path == '/poweroff':
            try:
                # Respond first, then shut down shortly after so the response can flush.
                self._send_json(200, {"status": "poweroff_scheduled"})
                if self.ros_node:
                    self.ros_node.get_logger().warn('[WIFI_RX] Poweroff requested from app.')
                    threading.Timer(1.0, self.ros_node.trigger_poweroff).start()
            except Exception as e:
                self.send_error(500, f"Server error: {e}")

        # ---- New: generate & publish global path ----
        elif self.path == '/global_path':
            try:
                params = self._read_json_body()
                if self.ros_node:
                    num = self.ros_node.set_global_path(params)
                    self._send_json(200, {"status": "ok", "num_waypoints": num})
                else:
                    self._send_json(500, {"status": "error", "msg": "ROS node not available"})
            except json.JSONDecodeError as e:
                self.send_error(400, f"Invalid JSON: {e}")
            except Exception as e:
                self.send_error(500, f"Server error: {e}")

        else:
            self.send_error(404, "Not Found")

    # Handle HTTP OPTIONS requests to support CORS preflight checks.
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def log_message(self, format, *args):
        if self.ros_node:
            self.ros_node.get_logger().debug(f"HTTP: {format % args}")


class AppWiFiReceiver(Node):
    def __init__(self):
        super().__init__('app_wifi_receiver')

        # Publishers (existing)
        self.sw_bits_pub = self.create_publisher(String, '/app/sw_bits', 10)
        self.key_bits_pub = self.create_publisher(String, '/app/key_bits', 10)
        self.speed_bits_pub = self.create_publisher(String, '/app/speed_bits', 10)
        self.raw_app_data_pub = self.create_publisher(String, '/app/raw_data', 10)
        self.video_bit_pub = self.create_publisher(String, '/app/video_bit', 10)
        self.safe_bit_pub = self.create_publisher(String, '/app/safe_bit', 10)

        # Publisher (new): global path generated from the app
        self.global_path_pub = self.create_publisher(Path, '/global_path', 10)
        self.current_path_msg = None
        # Re-publish the latest path at 1 Hz so late subscribers (PPC/EKF) still get it.
        self.create_timer(1.0, self._republish_path)

        # HTTP server settings
        self.declare_parameter('port', 8889)
        self.declare_parameter('host', '0.0.0.0')
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value

        # Stats
        self.data_count = 0
        self.last_data_time = None

        self.start_http_server()
        self.get_logger().info(f'[WIFI_RX] App WiFi Receiver started on {self.host}:{self.port}')
        self.get_logger().info('[WIFI_RX] POST endpoints: /to_rasp  /poweroff  /global_path')

    # ----------------------------------------------------------------- HTTP
    def start_http_server(self):
        def handler_factory(*args, **kwargs):
            return AppDataHandler(*args, ros_node=self, **kwargs)
        try:
            self.server = HTTPServer((self.host, self.port), handler_factory)
            self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self.server_thread.start()
            self.get_logger().info(f'[WIFI_RX] HTTP Server listening on {self.host}:{self.port}')
        except Exception as e:
            self.get_logger().error(f'[WIFI_RX] Failed to start HTTP server: {e}')

    # ------------------------------------------------------- command bits
    def process_app_data(self, json_data):
        try:
            self.data_count += 1
            current_time = self.get_clock().now()

            if 'sw_bits' in json_data:
                m = String(); m.data = str(json_data['sw_bits']); self.sw_bits_pub.publish(m)
            if 'key_bits' in json_data:
                m = String(); m.data = str(json_data['key_bits']); self.key_bits_pub.publish(m)
            if 'speed_bits' in json_data:
                m = String(); m.data = str(json_data['speed_bits']); self.speed_bits_pub.publish(m)
            if 'video_bit' in json_data:
                m = String(); m.data = str(json_data['video_bit']); self.video_bit_pub.publish(m)
            if 'safe_bit' in json_data:
                m = String(); m.data = str(json_data['safe_bit']); self.safe_bit_pub.publish(m)

            self.last_data_time = current_time
        except Exception as e:
            self.get_logger().error(f'[WIFI_RX] Error processing app data: {e}')

    # ------------------------------------------------------------ poweroff
    def trigger_poweroff(self):
        try:
            self.get_logger().warn('[WIFI_RX] Executing: sudo poweroff')
            subprocess.Popen(['sudo', 'poweroff'])
        except Exception as e:
            self.get_logger().error(f'[WIFI_RX] poweroff failed: {e}')

    # --------------------------------------------------------- global path
    def set_global_path(self, params):
        """Compute boustrophedon waypoints from app params and publish as nav_msgs/Path."""
        waypoints = self.compute_boustrophedon(params)
        self.current_path_msg = self.build_path_msg(waypoints)
        self._republish_path()  # publish immediately
        self.get_logger().info(f'[WIFI_RX] Global path set: {len(waypoints)} waypoints.')
        return len(waypoints)

    def compute_boustrophedon(self, params):
        """Port of global_path_generator.generate_boustrophedon_path() without input()/matplotlib."""
        def f(key, default):
            try:
                return float(params.get(key, default))
            except (TypeError, ValueError):
                return float(default)

        map_top_y    = f('map_top_y', 10.0)
        map_bottom_y = f('map_bottom_y', -10.0)
        field_top_y    = f('field_top_y', 8.0)
        field_bottom_y = f('field_bottom_y', -8.0)

        crop_raw = params.get('crop_x', "-6.6 -3.3 0 3.3 6.6")
        if isinstance(crop_raw, (list, tuple)):
            crop_x_list = [float(x) for x in crop_raw]
        else:
            crop_x_list = [float(x) for x in str(crop_raw).split()]
        crop_x_list.sort()

        waypoint_top_y = (map_top_y + field_top_y) / 2.0
        waypoint_bottom_y = (map_bottom_y + field_bottom_y) / 2.0

        if len(crop_x_list) > 1:
            interval = crop_x_list[1] - crop_x_list[0]
        else:
            interval = 5.0

        path_x_list = []
        path_x_list.append(crop_x_list[0] - interval / 2.0)
        for i in range(len(crop_x_list) - 1):
            path_x_list.append((crop_x_list[i] + crop_x_list[i + 1]) / 2.0)
        path_x_list.append(crop_x_list[-1] + interval / 2.0)

        corner_waypoints = []
        is_top = False
        for x in path_x_list:
            if not is_top:
                corner_waypoints.append((x, waypoint_bottom_y))
                corner_waypoints.append((x, waypoint_top_y))
            else:
                corner_waypoints.append((x, waypoint_top_y))
                corner_waypoints.append((x, waypoint_bottom_y))
            is_top = not is_top
        return corner_waypoints

    def build_path_msg(self, waypoints):
        path = Path()
        path.header.frame_id = 'map'
        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _republish_path(self):
        if self.current_path_msg is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.current_path_msg.header.stamp = stamp
        for pose in self.current_path_msg.poses:
            pose.header.stamp = stamp
        self.global_path_pub.publish(self.current_path_msg)

    # ------------------------------------------------------------- cleanup
    def destroy_node(self):
        self.get_logger().info('[WIFI_RX] Shutting down App WiFi Receiver...')
        if hasattr(self, 'server'):
            self.server.shutdown()
            self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AppWiFiReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()