#!/usr/bin/env python3
"""
Web joystick + WebSocket control node for the Orion robot dog.

Chay node nay tren robot (Jetson), roi mo dia chi duoc in ra tren dien thoai
(cung mang Wi-Fi) de dieu khien bang joystick ao.

Data path:
  Phone browser --HTTP--> this node (serves the joystick page)
  Phone browser --WebSocket(JSON)--> this node --/joy_motion_cmd--> cmd_mux
      --/orion_motion_cmd--> stm32_tcp_bridge_node --TCP--> ESP32 --> STM32
  /imu_degrees --> this node --WebSocket--> phone (hien thi IMU)

Client message (sent 20 Hz by the page), joystick values are -1..1:
  {"cmd_type": "NORMAL", "lin_x": 0.5, "lin_y": 0.0, "ang_z": -0.2}
The node multiplies them by max_lin_x / max_lin_y / max_ang_z.
cmd_type is a name: NORMAL, WAVE, EYES ... -> OrionMotionCmd.CMD_<NAME>.
"""

import asyncio
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import websockets
from rclpy.node import Node

from orion_msgs.msg import OrionMotionCmd, OrionImuFeedback


# ======================================================================
# Web page (served at http://<robot-ip>:<http_port>/)
# ======================================================================
HTML_PAGE = r"""<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
<title>Orion Control</title>
<style>
  :root {
    --bg: #10141a; --panel: #1b222c; --line: #2c3746;
    --text: #e8edf2; --muted: #8a96a3; --accent: #3da9fc;
    --ok: #2ecc71; --bad: #e74c3c; --warn: #f5a623;
    --s: min(42vw, 38vh, 230px);
  }
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  html, body {
    margin: 0; height: 100%; background: var(--bg); color: var(--text);
    font-family: system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
    overscroll-behavior: none; touch-action: none;
    user-select: none; -webkit-user-select: none;
  }
  body {
    display: flex; flex-direction: column; gap: 12px;
    padding: max(10px, env(safe-area-inset-top)) 12px max(14px, env(safe-area-inset-bottom));
  }
  .top { display: flex; align-items: center; justify-content: space-between; gap: 8px; font-size: 14px; }
  .conn { display: flex; align-items: center; gap: 8px; }
  .dot { width: 12px; height: 12px; border-radius: 50%; background: var(--bad); }
  .dot.on { background: var(--ok); }
  .imu { color: var(--muted); font-variant-numeric: tabular-nums; font-size: 13px; text-align: right; }
  .controls { display: flex; flex-wrap: wrap; gap: 10px; justify-content: center; }
  button {
    font: inherit; font-weight: 600; color: var(--text); background: var(--panel);
    border: 2px solid var(--line); border-radius: 12px; padding: 12px 18px; min-width: 96px;
  }
  button.mode.sel { border-color: var(--accent); color: var(--accent); }
  #arm.on { background: var(--ok); border-color: var(--ok); color: #06210f; }
  #stop { background: var(--bad); border-color: var(--bad); color: #fff; }
  .hint { text-align: center; color: var(--muted); font-size: 13px; min-height: 18px; }
  .sticks { margin-top: auto; display: flex; justify-content: space-around; align-items: flex-end; }
  .stickwrap { display: flex; flex-direction: column; align-items: center; gap: 8px; }
  .label { color: var(--muted); font-size: 13px; }
  .stick {
    width: var(--s); height: var(--s); border-radius: 50%;
    background: radial-gradient(circle at center, #222c39 0%, var(--panel) 70%);
    border: 2px solid var(--line); position: relative; touch-action: none;
  }
  .stick.h { border-radius: calc(var(--s) / 2); height: calc(var(--s) * 0.45); }
  .knob {
    position: absolute; left: 50%; top: 50%; width: 38%; aspect-ratio: 1;
    margin-left: -19%; border-radius: 50%; background: var(--accent);
    transform: translate(0, 0); box-shadow: 0 2px 10px rgba(0, 0, 0, .5);
  }
  .stick.h .knob { width: calc(var(--s) * 0.38 * 0.9); margin-left: calc(var(--s) * -0.19 * 0.9); }
  .stick .knob { margin-top: calc(var(--s) * -0.19); }
  .stick.h .knob { margin-top: calc(var(--s) * -0.19 * 0.9); }
  @media (orientation: landscape) and (max-height: 520px) {
    :root { --s: min(34vw, 62vh, 220px); }
    body { gap: 6px; }
  }
</style>
</head>
<body>
  <div class="top">
    <div class="conn"><span id="dot" class="dot"></span><span id="status">Dang ket noi...</span></div>
    <div class="imu" id="imu">IMU: --</div>
  </div>

  <div class="controls">
    <button id="arm">ARM: TAT</button>
    <button id="stop">STOP</button>
    <button class="mode sel" data-mode="NORMAL">Normal</button>
    <button class="mode" data-mode="WAVE">Wave</button>
  </div>
  <div class="hint" id="hint">Bam ARM de bat dieu khien. Tha joystick = robot dung.</div>

  <div class="sticks">
    <div class="stickwrap">
      <div class="stick" id="stickL"><div class="knob"></div></div>
      <div class="label">Tien / Lui / Sang ngang</div>
    </div>
    <div class="stickwrap">
      <div class="stick h" id="stickR"><div class="knob"></div></div>
      <div class="label">Xoay trai / phai</div>
    </div>
  </div>

<script>
"use strict";
const WS_PORT = __WS_PORT__;
const SEND_MS = 50;      // 20 Hz
const DEADZONE = 0.08;

const state = { fwd: 0, side: 0, turn: 0, mode: "NORMAL", armed: false };
let ws = null;

const $ = (id) => document.getElementById(id);

// ---------- WebSocket ----------
function connect() {
  ws = new WebSocket(`ws://${location.hostname}:${WS_PORT}`);
  ws.onopen = () => {
    $("dot").classList.add("on");
    $("status").textContent = "Da ket noi robot";
  };
  ws.onclose = () => {
    $("dot").classList.remove("on");
    $("status").textContent = "Mat ket noi - dang thu lai...";
    setArmed(false);
    setTimeout(connect, 1000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (e) => {
    let m;
    try { m = JSON.parse(e.data); } catch (_) { return; }
    if (m.type === "imu") {
      $("imu").textContent =
        `R ${m.roll.toFixed(1)}  P ${m.pitch.toFixed(1)}  Y ${m.yaw.toFixed(1)}`;
    } else if (m.type === "error") {
      $("hint").textContent = "Loi: " + m.msg;
    }
  };
}

function dz(v) {
  if (Math.abs(v) < DEADZONE) return 0;
  const s = (Math.abs(v) - DEADZONE) / (1 - DEADZONE);   // rescale after deadzone
  return Math.sign(v) * Math.min(1, s);
}
const r3 = (v) => Math.round(v * 1000) / 1000 + 0;       // +0 avoids -0

setInterval(() => {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  const live = state.armed;
  ws.send(JSON.stringify({
    cmd_type: state.mode,
    lin_x: live ? r3(dz(state.fwd)) : 0,
    lin_y: live ? r3(dz(-state.side)) : 0,   // ROS: +y = trai
    ang_z: live ? r3(dz(-state.turn)) : 0,   // ROS: +z = quay trai
  }));
}, SEND_MS);

// ---------- Joystick ----------
function makeStick(el, axis, onMove) {
  const knob = el.querySelector(".knob");
  let pid = null;

  function update(e) {
    const r = el.getBoundingClientRect();
    const cx = r.left + r.width / 2, cy = r.top + r.height / 2;
    const lim = (axis === "x" ? r.width : Math.min(r.width, r.height)) * 0.5 * 0.62;
    let dx = e.clientX - cx, dy = e.clientY - cy;
    if (axis === "x") dy = 0;
    const d = Math.hypot(dx, dy);
    if (d > lim) { dx *= lim / d; dy *= lim / d; }
    knob.style.transform = `translate(${dx}px, ${dy}px)`;
    onMove(dx / lim, -dy / lim);   // y up = positive
  }
  function end(e) {
    if (e.pointerId !== pid) return;
    pid = null;
    knob.style.transform = "translate(0, 0)";
    onMove(0, 0);
  }

  el.addEventListener("pointerdown", (e) => {
    if (pid !== null) return;
    pid = e.pointerId;
    el.setPointerCapture(pid);
    update(e);
    e.preventDefault();
  });
  el.addEventListener("pointermove", (e) => { if (e.pointerId === pid) update(e); });
  el.addEventListener("pointerup", end);
  el.addEventListener("pointercancel", end);
}

makeStick($("stickL"), "xy", (x, y) => { state.side = x; state.fwd = y; });
makeStick($("stickR"), "x", (x) => { state.turn = x; });

// ---------- Buttons ----------
function setArmed(on) {
  state.armed = on;
  $("arm").textContent = on ? "ARM: BAT" : "ARM: TAT";
  $("arm").classList.toggle("on", on);
  $("hint").textContent = on
    ? "Dang dieu khien. Tha joystick = robot dung."
    : "Bam ARM de bat dieu khien.";
}
$("arm").addEventListener("click", () => setArmed(!state.armed));
$("stop").addEventListener("click", () => setArmed(false));

document.querySelectorAll("button.mode").forEach((b) => {
  b.addEventListener("click", () => {
    state.mode = b.dataset.mode;
    document.querySelectorAll("button.mode").forEach((x) => x.classList.toggle("sel", x === b));
  });
});

// Roi khoi trang / an man hinh -> tu dong tat ARM
document.addEventListener("visibilitychange", () => { if (document.hidden) setArmed(false); });
window.addEventListener("blur", () => setArmed(false));

connect();
</script>
</body>
</html>
"""


# ======================================================================
# Helpers
# ======================================================================
def get_lan_ips():
    """Return non-loopback IPv4 addresses of this machine."""
    ips = []

    # Primary outgoing interface (no packet is actually sent).
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass

    # Fallback: every address bound to this hostname.
    try:
        for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
            if ip not in ips:
                ips.append(ip)
    except OSError:
        pass

    return [ip for ip in ips if not ip.startswith("127.")]


def make_page_handler(page_bytes):
    class PageHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.split("?")[0] in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page_bytes)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(page_bytes)
            else:
                self.send_error(404)

        def log_message(self, *args):
            pass  # keep the ROS console clean

    return PageHandler


# ======================================================================
# Node
# ======================================================================
class WebSocketControl(Node):
    def __init__(self, loop):
        super().__init__("websocket_control_node")

        # ---------- Parameters ----------
        self.declare_parameter("ws_port", 8765)
        self.declare_parameter("http_port", 8000)
        self.declare_parameter("cmd_timeout_sec", 0.3)
        self.declare_parameter("imu_push_rate_hz", 10.0)
        # Joystick (-1..1) is multiplied by these before publishing.
        self.declare_parameter("max_lin_x", 1.0)
        self.declare_parameter("max_lin_y", 1.0)
        self.declare_parameter("max_ang_z", 1.0)

        self.ws_port = int(self.get_parameter("ws_port").value)
        self.http_port = int(self.get_parameter("http_port").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        imu_rate = float(self.get_parameter("imu_push_rate_hz").value)
        self.imu_period = 1.0 / imu_rate if imu_rate > 0 else 0.1
        self.max_lin_x = float(self.get_parameter("max_lin_x").value)
        self.max_lin_y = float(self.get_parameter("max_lin_y").value)
        self.max_ang_z = float(self.get_parameter("max_ang_z").value)

        # NOTE: do not name this `clients` - rclpy.node.Node already has a
        # read-only `clients` property (the node's service clients).
        self.loop = loop
        self.ws_clients = set()
        self.last_rx = 0.0
        self.stopped = True
        self.last_imu_push = 0.0
        self.httpd = None

        # ---------- ROS interfaces ----------
        # Publish as a "joystick" source; cmd_mux forwards it to orion_motion_cmd.
        self.cmd_pub = self.create_publisher(
            OrionMotionCmd, "joy_motion_cmd", 10
        )
        self.create_subscription(
            OrionImuFeedback, "imu_degrees", self.imu_callback, 10
        )
        self.create_timer(0.05, self.watchdog)  # 20 Hz

    # ------------------------------------------------------------------
    # HTTP server for the web page
    # ------------------------------------------------------------------
    def start_http_server(self):
        page = HTML_PAGE.replace("__WS_PORT__", str(self.ws_port)).encode("utf-8")
        try:
            self.httpd = ThreadingHTTPServer(
                ("0.0.0.0", self.http_port), make_page_handler(page)
            )
        except OSError as exc:
            self.get_logger().error(
                f"Cannot start web page server on port {self.http_port}: {exc}"
            )
            return
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def print_access_info(self):
        ips = get_lan_ips()
        urls = [f"http://{ip}:{self.http_port}" for ip in ips]
        try:
            urls.append(f"http://{socket.gethostname()}.local:{self.http_port}")
        except OSError:
            pass

        line = "=" * 62
        print(line, flush=True)
        print(" DIEU KHIEN ROBOT BANG DIEN THOAI", flush=True)
        print(" 1) Noi dien thoai cung mang Wi-Fi voi robot", flush=True)
        print(" 2) Mo Chrome tren dien thoai, go mot trong cac dia chi:", flush=True)
        for url in urls:
            print(f"      {url}", flush=True)
        print(" 3) Bam ARM, roi dung joystick de dieu khien", flush=True)
        print(line, flush=True)

        # Optional QR code: pip install qrcode  (or apt install python3-qrcode)
        if ips:
            try:
                import qrcode

                qr = qrcode.QRCode(border=1)
                qr.add_data(f"http://{ips[0]}:{self.http_port}")
                qr.make(fit=True)
                print(" Hoac quet ma QR:", flush=True)
                qr.print_ascii(invert=True)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # WebSocket -> ROS
    # ------------------------------------------------------------------
    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))

    def handle_command(self, data):
        if not isinstance(data, dict):
            raise ValueError("command must be a JSON object")

        msg = OrionMotionCmd()

        cmd = data.get("cmd_type", "NORMAL")
        if isinstance(cmd, str):
            const = getattr(OrionMotionCmd, "CMD_" + cmd.upper(), None)
            if const is None:
                raise ValueError(f"unknown cmd_type '{cmd}'")
            msg.cmd_type = int(const)
        else:
            msg.cmd_type = int(cmd)

        msg.lin_x = self._clamp(float(data.get("lin_x", 0.0)), -1.0, 1.0) * self.max_lin_x
        msg.lin_y = self._clamp(float(data.get("lin_y", 0.0)), -1.0, 1.0) * self.max_lin_y
        msg.ang_z = self._clamp(float(data.get("ang_z", 0.0)), -1.0, 1.0) * self.max_ang_z

        self.cmd_pub.publish(msg)
        self.last_rx = time.monotonic()
        self.stopped = False

    def watchdog(self):
        # Lost the client -> publish a zero command once.
        if not self.stopped and time.monotonic() - self.last_rx > self.cmd_timeout:
            stop = OrionMotionCmd()
            stop.cmd_type = OrionMotionCmd.CMD_NORMAL
            self.cmd_pub.publish(stop)
            self.stopped = True
            self.get_logger().warning("Command timeout -> sent zero command")

    # ------------------------------------------------------------------
    # ROS -> WebSocket
    # ------------------------------------------------------------------
    def imu_callback(self, msg):
        now = time.monotonic()
        if now - self.last_imu_push < self.imu_period:
            return
        self.last_imu_push = now

        if not self.ws_clients:
            return

        text = json.dumps(
            {
                "type": "imu",
                "roll": msg.roll,
                "pitch": msg.pitch,
                "yaw": msg.yaw,
            }
        )
        asyncio.run_coroutine_threadsafe(self.broadcast(text), self.loop)

    async def broadcast(self, text):
        for ws in list(self.ws_clients):
            try:
                await ws.send(text)
            except websockets.ConnectionClosed:
                self.ws_clients.discard(ws)

    # ------------------------------------------------------------------
    # WebSocket server
    # ------------------------------------------------------------------
    async def handler(self, ws):
        self.ws_clients.add(ws)
        self.get_logger().info("WebSocket client connected")
        try:
            async for raw in ws:
                try:
                    self.handle_command(json.loads(raw))
                except (ValueError, TypeError) as exc:
                    await ws.send(json.dumps({"type": "error", "msg": str(exc)}))
        except websockets.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(ws)
            self.get_logger().info("WebSocket client disconnected")


def main(args=None):
    rclpy.init(args=args)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    node = WebSocketControl(loop)

    # rclpy spins in a background thread; asyncio runs in the main thread.
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    node.start_http_server()

    async def serve():
        async with websockets.serve(node.handler, "0.0.0.0", node.ws_port):
            node.get_logger().info(
                f"WebSocket server on port {node.ws_port}, web page on port {node.http_port}"
            )
            node.print_access_info()
            await asyncio.Future()  # run forever

    try:
        loop.run_until_complete(serve())
    except KeyboardInterrupt:
        pass
    finally:
        if node.httpd is not None:
            node.httpd.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()