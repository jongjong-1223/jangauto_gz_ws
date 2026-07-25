#!/usr/bin/env python3
"""앱 웹소켓 브릿지 노드.

## 역할
- 모바일 앱과 단일 양방향 WebSocket(`ws://<host>:8887`)으로 연결해, 앱이
  보낸 JSON을 그대로 ROS 토픽으로 재발행한다(`/app/control_state`,
  `/app/command`).
- `mission_state_machine.py`가 발행하는 앱 핸드셰이크 토픽
  (`/app/robot_status`, `/app/control_state_ack`)을 구독해서 연결된 모든
  웹소켓 클라이언트에 그대로 중계한다 — 내용을 해석하지 않는 "덤(dumb)
  중계"만 담당하고, 수락/거부 판단이나 상태 전이 로직은 전혀 갖지 않는다
  (전부 `mission_state_machine.py` 책임).
- mDNS(`_robot._tcp.local.`)로 서비스를 광고해서 앱이 IP를 몰라도 자동
  탐색할 수 있게 한다.
- 하트비트(`/app/link_alive`)로 앱 연결 생존 여부를 추적하고,
  `diagnostic_updater`로 서버/연결 상태를 `/diagnostics`에 보고한다.

## 스코프 밖
JSON을 실제 로봇 명령(`cmd_vel`, poweroff, generate_path 등)으로 번역하는
로직은 여기 없다 — `app_bridge.py`/`app_wifi_rx.py`/`app_wifi_tx.py`(구
HTTP 프로토콜용 참고 코드, 현재 미사용)가 하던 역할이지만 이 노드는 그걸
가져다 쓰지 않는다. 지금 프로토콜(단일 WebSocket, 0-padding 문자열 대신
정수 비트필드)이 다르기 때문에 새로 만들었다.
"""
import asyncio
import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool

import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus

import websockets
from websockets.exceptions import ConnectionClosed

from zeroconf import Zeroconf, ServiceInfo

# Keys present in the periodic control-state payload the app sends every
# Config.TX_PERIOD_MS (500 ms by default on the app side).
CONTROL_STATE_KEYS = {'sw_bits', 'key_bits', 'speed_bits', 'video_bit', 'safe_bit'}

# Topics mission_state_machine.py publishes for the app handshake — this
# bridge relays them to the app verbatim without interpreting their content
# (accept/reject and sequencing logic all live in mission_state_machine.py).
APP_ROBOT_STATUS_TOPIC = '/app/robot_status'
APP_CONTROL_STATE_ACK_TOPIC = '/app/control_state_ack'

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

        # Subscriptions: mission_state_machine.py -> app handshake relay.
        # Cache the latest robot_status so newly-connected clients get it
        # immediately instead of waiting for the next state change (mirrors
        # /robot_status's own TRANSIENT_LOCAL/latched behavior at the
        # WebSocket layer, where ROS QoS latching doesn't reach).
        self._last_robot_status_json = None
        # mission_state_machine.py가 이 토픽을 RELIABLE+TRANSIENT_LOCAL(latched)로
        # 발행한다 — 이쪽 구독도 durability를 맞춰야 late-join 시 마지막 값을 실제로
        # 받는다(구독 쪽이 기본 VOLATILE이면 QoS는 호환되어 에러는 안 나지만, 늦게
        # 구독해도 과거 값을 재생해주지 않고 그 다음 변화부터만 받게 됨 — 실행 확인됨).
        app_status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, APP_ROBOT_STATUS_TOPIC, self._on_app_robot_status, app_status_qos)
        self.create_subscription(
            String, APP_CONTROL_STATE_ACK_TOPIC, self._on_app_control_state_ack, 10)

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

        self._diag_updater = diagnostic_updater.Updater(self)
        self._diag_updater.setHardwareID('app_websocket_bridge')
        self._diag_updater.add('WebSocket link', self._diagnostics_callback)

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

        - 접속하면 클라이언트 집합에 등록하고, 캐시된 최신 상태가 있으면
          바로 한 번 보내준다(늦게 접속해도 현재 상태를 즉시 알 수 있게).
        - 이후 들어오는 메시지마다 `_on_message`로 넘긴다.
        - 어떤 예외가 나든(연결 끊김 포함) 다른 클라이언트에 영향 없이
          이 커넥션만 정리하고 끝낸다.
        """
        peer = websocket.remote_address
        self.get_logger().info(f'[AppWsBridge] Client connected: {peer}')
        with self._ws_lock:
            self._ws_clients.add(websocket)
        if self._last_robot_status_json is not None:
            try:
                await websocket.send(self._last_robot_status_json)
            except Exception as e:
                self.get_logger().warn(f'[AppWsBridge] Failed to send initial status to {peer}: {e}')
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
        elif data.keys() & CONTROL_STATE_KEYS:
            out = String()
            out.data = json.dumps(data)
            self.control_state_pub.publish(out)
        else:
            self.get_logger().warn(
                f'[AppWsBridge] Unrecognized message shape from {peer}: {list(data.keys())}',
                throttle_duration_sec=5.0)

    # ------------------------------------------------------ app handshake relay
    def _on_app_robot_status(self, msg: String) -> None:
        """`/app/robot_status` 구독 콜백 — 최신 값을 캐시해두고(신규 접속
        클라이언트용) 지금 붙어있는 클라이언트 전체에 그대로 중계한다."""
        self._last_robot_status_json = msg.data
        self._broadcast_to_clients(msg.data)

    def _on_app_control_state_ack(self, msg: String) -> None:
        """`/app/control_state_ack` 구독 콜백 — 캐싱 없이 그대로 중계한다
        (매 명령마다 오는 1회성 응답이라 재접속 시 다시 보내줄 대상이 아님)."""
        self._broadcast_to_clients(msg.data)

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

    # ---------------------------------------------------------------- cleanup
    def destroy_node(self):
        """노드 종료 시 mDNS 등록 해제 + 웹소켓 서버 정리까지 마친 뒤
        상위 `Node.destroy_node()`를 호출한다."""
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
