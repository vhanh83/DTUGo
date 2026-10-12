#!/usr/bin/env python3
"""
ROS 2 Jazzy TCP bridge for STM32F411 via ESP32-S3.

Data path:
  ROS 2 command topic -> this node -> TCP/Wi-Fi -> ESP32-S3 -> UART -> STM32
  STM32 telemetry -> UART -> ESP32-S3 -> TCP/Wi-Fi -> this node -> ROS 2 topics

TCP is a byte stream, so this node buffers received data and searches for
telemetry frames. The binary STM32 packet format is kept unchanged.
"""

import math
import socket
import struct
import time

import rclpy
from rclpy.node import Node

from orion_msgs.msg import (
    OrionMotionCmd,
    OrionImuFeedback,
    OrionBatteryVoltage,
    OrionLegInfo,
)
from sensor_msgs.msg import JointState


# Telemetry frame:
# 0xAA 0x55 + 24 little-endian floats (96 bytes) + XOR checksum = 99 bytes
TELEM_HEADER = b"\xAA\x55"
TELEM_PKT_SIZE = 99

# Command frame:
# 0x55 0xAA + uint8 command type + 11 float32 + XOR checksum = 48 bytes
CMD_HEADER = b"\x55\xAA"
CMD_PAYLOAD_FORMAT = "<B11f"
CMD_PAYLOAD_SIZE = struct.calcsize(CMD_PAYLOAD_FORMAT)
CMD_PKT_SIZE = 2 + CMD_PAYLOAD_SIZE + 1


class STM32TcpBridge(Node):
    def __init__(self):
        super().__init__("stm32_tcp_bridge_node")

        # ---------- Parameters ----------
        self.declare_parameter("esp32_ip", "192.168.5.227")
        self.declare_parameter("esp32_port", 5000)
        self.declare_parameter("send_rate_hz", 50.0)
        self.declare_parameter("reconnect_interval_sec", 2.0)

        self.esp32_ip = str(self.get_parameter("esp32_ip").value)
        self.esp32_port = int(self.get_parameter("esp32_port").value)
        send_rate_hz = float(self.get_parameter("send_rate_hz").value)
        self.reconnect_interval_sec = float(
            self.get_parameter("reconnect_interval_sec").value
        )

        if send_rate_hz <= 0:
            raise ValueError("send_rate_hz must be greater than zero")

        self.send_period = 1.0 / send_rate_hz
        self.sock = None
        self.rx_buffer = bytearray()
        self.tx_pending = bytearray()
        self.last_connect_attempt = 0.0
        self.last_connection_log = 0.0
        self.last_checksum_warning = 0.0

        # ---------- ROS command subscriber ----------
        self.latest_msg = OrionMotionCmd()
        self.latest_msg.cmd_type = OrionMotionCmd.CMD_NORMAL

        self.motion_subscriber = self.create_subscription(
            OrionMotionCmd,
            "orion_motion_cmd",
            self.motion_callback,
            10,
        )

        # ---------- Telemetry publishers ----------
        self.battery_front_pub = self.create_publisher(
            OrionBatteryVoltage, "battery_voltage_front", 10
        )
        self.battery_rear_pub = self.create_publisher(
            OrionBatteryVoltage, "battery_voltage_rear", 10
        )
        self.battery_jetson_pub = self.create_publisher(
            OrionBatteryVoltage, "battery_voltage_jetson", 10
        )

        self.imu_pub = self.create_publisher(
            OrionImuFeedback, "imu_degrees", 10
        )

        self.front_left_leg_pub = self.create_publisher(
            OrionLegInfo, "joint_angles_front_left", 10
        )
        self.front_right_leg_pub = self.create_publisher(
            OrionLegInfo, "joint_angles_front_right", 10
        )
        self.back_left_leg_pub = self.create_publisher(
            OrionLegInfo, "joint_angles_back_left", 10
        )
        self.back_right_leg_pub = self.create_publisher(
            OrionLegInfo, "joint_angles_back_right", 10
        )

        self.joint_state_pub = self.create_publisher(
            JointState, "joint_states", 10
        )

        # ---------- Timers ----------
        self.connect_timer = self.create_timer(0.2, self.try_connect)
        self.write_timer = self.create_timer(
            self.send_period, self.send_to_stm32
        )
        self.read_timer = self.create_timer(0.005, self.read_from_stm32)

        self.get_logger().info(
            f"TCP bridge created; ESP32-S3={self.esp32_ip}:{self.esp32_port}, "
            f"command rate={send_rate_hz:.1f} Hz"
        )
        self.get_logger().info(
            f"STM32 command frame={CMD_PKT_SIZE} bytes; "
            f"telemetry frame={TELEM_PKT_SIZE} bytes"
        )

    # ------------------------------------------------------------------
    # TCP connection management
    # ------------------------------------------------------------------
    def try_connect(self):
        if self.sock is not None:
            return

        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_interval_sec:
            return
        self.last_connect_attempt = now

        candidate = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        candidate.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        candidate.settimeout(0.75)

        try:
            candidate.connect((self.esp32_ip, self.esp32_port))
            candidate.setblocking(False)
            self.sock = candidate
            self.rx_buffer.clear()
            self.tx_pending.clear()
            self.get_logger().info(
                f"Connected to ESP32-S3 at {self.esp32_ip}:{self.esp32_port}"
            )
        except (OSError, TimeoutError) as exc:
            candidate.close()
            if now - self.last_connection_log >= 5.0:
                self.get_logger().warning(
                    f"Cannot connect to ESP32-S3 "
                    f"{self.esp32_ip}:{self.esp32_port}: {exc}"
                )
                self.last_connection_log = now

    def disconnect(self, reason):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.rx_buffer.clear()
        self.tx_pending.clear()
        self.get_logger().warning(f"TCP disconnected: {reason}; will reconnect")

    # ------------------------------------------------------------------
    # ROS command -> STM32 binary frame -> TCP
    # ------------------------------------------------------------------
    def motion_callback(self, msg):
        self.latest_msg = msg

    @staticmethod
    def calculate_checksum(payload_bytes):
        checksum = 0
        for value in payload_bytes:
            checksum ^= value
        return checksum

    def make_command_packet(self):
        msg = self.latest_msg

        payload = struct.pack(
            CMD_PAYLOAD_FORMAT,
            int(msg.cmd_type),
            float(msg.lin_x),
            float(msg.lin_y),
            float(msg.ang_z),
            float(msg.roll),
            float(msg.pitch),
            float(msg.yaw),
            float(msg.z_offset),
            float(msg.x_offset),
            float(msg.y_offset),
            float(msg.pivot_x),
            float(msg.pivot_y),
        )

        checksum = self.calculate_checksum(payload)
        return CMD_HEADER + payload + bytes((checksum,))

    def flush_pending_tx(self):
        """Send any remainder of a partially sent TCP frame."""
        if self.sock is None or not self.tx_pending:
            return

        try:
            sent = self.sock.send(self.tx_pending)
            if sent > 0:
                del self.tx_pending[:sent]
        except BlockingIOError:
            pass
        except OSError as exc:
            self.disconnect(f"send failed: {exc}")

    def send_to_stm32(self):
        if self.sock is None:
            return

        # A partially sent frame must be completed before another frame is
        # started; otherwise the binary protocol stream would be corrupted.
        self.flush_pending_tx()
        if self.sock is None or self.tx_pending:
            return

        try:
            packet = self.make_command_packet()
            sent = self.sock.send(packet)
            if sent < len(packet):
                self.tx_pending.extend(packet[sent:])
        except BlockingIOError:
            # Socket is temporarily not writable; try again next timer tick.
            # No bytes were accepted by send() in this case.
            pass
        except OSError as exc:
            self.disconnect(f"send failed: {exc}")

    # ------------------------------------------------------------------
    # TCP telemetry stream -> binary frames -> ROS publishers
    # ------------------------------------------------------------------
    def read_from_stm32(self):
        if self.sock is None:
            return

        try:
            # Drain currently available TCP data without blocking.
            while True:
                try:
                    chunk = self.sock.recv(2048)
                except BlockingIOError:
                    break

                if not chunk:
                    self.disconnect("peer closed connection")
                    return

                self.rx_buffer.extend(chunk)

                # Avoid unbounded memory growth if the stream is corrupted.
                if len(self.rx_buffer) > 8192:
                    del self.rx_buffer[:-TELEM_PKT_SIZE]

        except OSError as exc:
            self.disconnect(f"receive failed: {exc}")
            return

        self.process_telemetry_buffer()

    def process_telemetry_buffer(self):
        while len(self.rx_buffer) >= 2:
            header_pos = self.rx_buffer.find(TELEM_HEADER)

            if header_pos < 0:
                # Preserve a possible first header byte at the end.
                keep_last = (
                    self.rx_buffer[-1:] if self.rx_buffer[-1] == TELEM_HEADER[0]
                    else b""
                )
                self.rx_buffer.clear()
                self.rx_buffer.extend(keep_last)
                return

            if header_pos > 0:
                del self.rx_buffer[:header_pos]

            if len(self.rx_buffer) < TELEM_PKT_SIZE:
                return

            packet = bytes(self.rx_buffer[:TELEM_PKT_SIZE])
            payload = packet[2:-1]
            received_checksum = packet[-1]
            calculated_checksum = self.calculate_checksum(payload)

            if calculated_checksum != received_checksum:
                now = time.monotonic()
                if now - self.last_checksum_warning >= 2.0:
                    self.get_logger().warning(
                        "Telemetry checksum failed; resynchronizing stream"
                    )
                    self.last_checksum_warning = now

                # Drop one byte only, then search for the next valid header.
                del self.rx_buffer[0]
                continue

            del self.rx_buffer[:TELEM_PKT_SIZE]

            try:
                floats = struct.unpack("<24f", payload)
            except struct.error as exc:
                self.get_logger().warning(f"Cannot decode telemetry: {exc}")
                continue

            self.publish_telemetry(floats)

    def publish_telemetry(self, floats):
        imu_data = floats[:3]
        battery_data = floats[3:12]
        joint_data = floats[12:24]

        # IMU
        imu_msg = OrionImuFeedback()
        imu_msg.roll = float(imu_data[0])
        imu_msg.pitch = float(imu_data[1])
        imu_msg.yaw = float(imu_data[2])
        self.imu_pub.publish(imu_msg)

        # Front battery: cumulative cell readings in the incoming packet
        front_msg = OrionBatteryVoltage()
        front_msg.cell_1 = float(battery_data[0])
        front_msg.cell_2 = float(battery_data[1] - battery_data[0])
        front_msg.total = float(battery_data[1])
        self.battery_front_pub.publish(front_msg)

        # Rear battery
        rear_msg = OrionBatteryVoltage()
        rear_msg.cell_1 = float(battery_data[3])
        rear_msg.cell_2 = float(battery_data[4] - battery_data[3])
        rear_msg.total = float(battery_data[4])
        self.battery_rear_pub.publish(rear_msg)

        # Jetson battery: cumulative cell readings in the incoming packet
        jetson_msg = OrionBatteryVoltage()
        jetson_msg.cell_1 = float(battery_data[6])
        jetson_msg.cell_2 = float(battery_data[7] - battery_data[6])
        jetson_msg.cell_3 = float(battery_data[8] - battery_data[7])
        jetson_msg.total = float(battery_data[8])
        self.battery_jetson_pub.publish(jetson_msg)

        # Four legs, three joint angles per leg. Custom messages keep degrees,
        # matching the existing STM32Bridge implementation.
        leg_messages = [
            (self.front_left_leg_pub, joint_data[0:3]),
            (self.front_right_leg_pub, joint_data[3:6]),
            (self.back_left_leg_pub, joint_data[6:9]),
            (self.back_right_leg_pub, joint_data[9:12]),
        ]

        for publisher, angles in leg_messages:
            leg_msg = OrionLegInfo()
            leg_msg.hip_angle = float(angles[0])
            leg_msg.femur_angle = float(angles[1])
            leg_msg.tibia_angle = float(angles[2])
            publisher.publish(leg_msg)

        # Standard JointState convention uses radians. The custom leg topics
        # above continue to carry degrees, as in the original node.
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            "front_left_hip", "front_left_femur", "front_left_tibia",
            "front_right_hip", "front_right_femur", "front_right_tibia",
            "back_left_hip", "back_left_femur", "back_left_tibia",
            "back_right_hip", "back_right_femur", "back_right_tibia",
        ]
        joint_msg.position = [math.radians(float(value)) for value in joint_data]
        self.joint_state_pub.publish(joint_msg)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32TcpBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
