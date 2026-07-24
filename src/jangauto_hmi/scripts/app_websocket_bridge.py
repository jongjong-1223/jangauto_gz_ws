#!/usr/bin/env python3
"""App WebSocket Bridge: receive JSON from the mobile app over a single
bidirectional WebSocket connection and republish it as ROS topics.

Phase 1 scope only: connection handling, robustness/safety, heartbeat
liveness tracking, and JSON -> topic forwarding. Translating the
forwarded JSON into actual robot commands (cmd_vel, poweroff,
generate_path, ...) and pushing status back to the app are left for a
later phase. app_bridge.py / app_wifi_rx.py / app_wifi_tx.py remain as
reference for that previous HTTP-based protocol and its logic, but are
not imported or reused here — the app now speaks a different protocol
(single WebSocket at ws://<host>:8887, integer bit fields instead of
zero-padded bit strings).
"""
import asyncio
import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

import websockets
from websockets.exceptions import ConnectionClosed

from zeroconf import Zeroconf, ServiceInfo

# Keys present in the periodic control-state payload the app sends every
# Config.TX_PERIOD_MS (500 ms by default on the app side).
CONTROL_STATE_KEYS = {'sw_bits', 'key_bits', 'speed_bits', 'video_bit', 'safe_bit'}

# Must match Config.SERVICE_TYPE ("_robot._tcp.") in the app's NsdHelper.kt,
# with the ".local." domain that Android's NsdManager appends implicitly but
# python-zeroconf requires spelled out.
MDNS_SERVICE_TYPE = '_robot._tcp.local.'


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
    def __init__(self):
        super().__init__('app_websocket_bridge')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8887)
        self.declare_parameter('heartbeat_period_sec', 0.5)
        self.declare_parameter('heartbeat_timeout_sec', 1.5)
        self.declare_parameter('mdns_enabled', True)
        self.declare_parameter('mdns_instance_name', 'jangauto')

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.heartbeat_period_sec = self.get_parameter('heartbeat_period_sec').get_parameter_value().double_value
        self.heartbeat_timeout_sec = self.get_parameter('heartbeat_timeout_sec').get_parameter_value().double_value
        self.mdns_enabled = self.get_parameter('mdns_enabled').get_parameter_value().bool_value
        self.mdns_instance_name = self.get_parameter('mdns_instance_name').get_parameter_value().string_value

        # Publishers
        self.control_state_pub = self.create_publisher(String, '/app/control_state', 10)
        self.command_pub = self.create_publisher(String, '/app/command', 10)
        self.link_alive_pub = self.create_publisher(Bool, '/app/link_alive', 10)

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
        self._ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._ws_loop)
        try:
            self._ws_loop.run_until_complete(self._serve())
        except Exception as e:
            self.get_logger().error(f'[AppWsBridge] WebSocket server crashed: {e}')

    async def _serve(self):
        self._ws_server = await websockets.serve(self._handle_client, self.host, self.port)
        self.get_logger().info(f'[AppWsBridge] WebSocket server listening on {self.host}:{self.port}')
        await self._ws_server.wait_closed()

    async def _handle_client(self, websocket, path=None):
        peer = websocket.remote_address
        self.get_logger().info(f'[AppWsBridge] Client connected: {peer}')
        with self._ws_lock:
            self._ws_clients.add(websocket)
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
        elif data.keys() & CONTROL_STATE_KEYS:
            out = String()
            out.data = json.dumps(data)
            self.control_state_pub.publish(out)
        else:
            self.get_logger().warn(
                f'[AppWsBridge] Unrecognized message shape from {peer}: {list(data.keys())}',
                throttle_duration_sec=5.0)

    # -------------------------------------------------------------- heartbeat
    def _heartbeat_check(self):
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

    # ---------------------------------------------------------------- cleanup
    def destroy_node(self):
        self.get_logger().info('[AppWsBridge] Shutting down App WebSocket Bridge...')
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
