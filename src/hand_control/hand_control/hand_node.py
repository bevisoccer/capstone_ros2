#!/usr/bin/env python3
"""
hand_node.py — ROS2 node for the robotic hand (Teensy 4.1 over serial)

Hardware: PCA9685 (14 servos) · 5x INA219 current sensors · 5x FSR force sensors

Publishes:
  /hand/fsr_pct       (std_msgs/Float32MultiArray)  — FSR force 0.0–100.0 %, 5 elements
                                                       [thumb, index, middle, ring, pinky]
  /hand/fsr_raw       (std_msgs/Int32MultiArray)    — FSR raw ADC 0–1023, 5 elements
  /hand/current_ma    (std_msgs/Float32MultiArray)  — finger current mA, 5 elements
  /hand/power_mw      (std_msgs/Float32MultiArray)  — finger power mW, 5 elements

Subscribes:
  /hand/finger_cmd    (std_msgs/Float32MultiArray)  — finger open/close 0.0–1.0, 5 elements
                                                       [thumb, index, middle, ring, pinky]
                                                       0.0 = fully open (home), 1.0 = fully closed

Parameters:
  serial_port    (string)  — default "/dev/ttyACM0"
  baud_rate      (int)     — default 115200
  fsr_hz         (float)   — FSR stream rate Hz, default 20
  ina_hz         (float)   — INA219 stream rate Hz, default 20
  open_angle     (float)   — servo angle for open position, default 0.0
  closed_angle   (float)   — servo angle for closed position, default 150.0
"""

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String

from hand_control.hand_control_interface import (
    HandController, FINGER_CHANNELS, FSR_FINGER, INA_FINGER
)

# Finger order used for all multi-array messages
FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]

# Reverse maps: finger name → index in FINGER_ORDER
_FINGER_IDX = {name: i for i, name in enumerate(FINGER_ORDER)}

# INA_FINGER maps index→finger; build finger→ina_index
_INA_IDX = {v: k for k, v in INA_FINGER.items()}

# FSR_FINGER maps index→finger; build finger→fsr_index
_FSR_IDX = {v: k for k, v in FSR_FINGER.items()}


class HandNode(Node):
    def __init__(self):
        super().__init__("hand_node")

        self.declare_parameter("serial_port",   "auto")
        self.declare_parameter("baud_rate",     115200)
        self.declare_parameter("fsr_hz",        20.0)
        self.declare_parameter("ina_hz",        20.0)
        self.declare_parameter("open_angle",    150.0)
        self.declare_parameter("closed_angle",  0.0)

        self._port         = self.get_parameter("serial_port").value
        self._baud         = self.get_parameter("baud_rate").value
        self._fsr_hz       = self.get_parameter("fsr_hz").value
        self._ina_hz       = self.get_parameter("ina_hz").value
        self._open_angle   = self.get_parameter("open_angle").value
        self._closed_angle = self.get_parameter("closed_angle").value

        # Publishers
        self._pub_fsr_pct    = self.create_publisher(Float32MultiArray, "/hand/fsr_pct",    10)
        self._pub_fsr_raw    = self.create_publisher(Int32MultiArray,   "/hand/fsr_raw",    10)
        self._pub_current_ma = self.create_publisher(Float32MultiArray, "/hand/current_ma", 10)
        self._pub_power_mw   = self.create_publisher(Float32MultiArray, "/hand/power_mw",   10)

        # Subscribers
        self._sub_finger_cmd = self.create_subscription(
            Float32MultiArray, "/hand/finger_cmd", self._finger_cmd_cb, 10)
        self._sub_command = self.create_subscription(
            String, "/hand/command", self._command_cb, 10)

        self._paused = True  # held until _connect() finishes opening fingers
        self._last_angles = [None] * 5   # last sent angle per finger
        self._angle_deadband = 2.0       # degrees — skip send if change < this

        # Thread-safe state buffers (indexed by FINGER_ORDER)
        self._lock         = threading.Lock()
        self._fsr_pct      = [0.0] * 5
        self._fsr_raw      = [0]   * 5
        self._current_ma   = [0.0] * 5
        self._power_mw     = [0.0] * 5

        # Hand controller
        self._ctrl = HandController(port=self._port, baud=self._baud)
        self._ctrl.on_fsr_update = self._on_fsr
        self._ctrl.on_ina_update = self._on_ina

        # Connect in background so node init doesn't block
        connect_thread = threading.Thread(target=self._connect, daemon=True)
        connect_thread.start()

        # Publish timer at the faster of the two stream rates
        pub_hz = max(self._fsr_hz, self._ina_hz)
        self.create_timer(1.0 / pub_hz, self._publish_all)

        self.get_logger().info(
            f"HandNode started — connecting to Teensy on {self._port}")

    # ── Connect ────────────────────────────────────────────────────────────────

    def _connect(self):
        if not self._ctrl.connect():
            self.get_logger().error(
                f"[HAND] Could not open serial port {self._port}. "
                "Check the port and baud rate parameters.")
            return

        # Override stream rates from parameters
        self._ctrl._send(f"STREAM {int(self._ina_hz)}")
        self._ctrl._send(f"FSRSTREAM {int(self._fsr_hz)}")
        self.get_logger().info(
            f"[HAND] Connected. INA@{int(self._ina_hz)}Hz  FSR@{int(self._fsr_hz)}Hz")

        self._paused = False
        self.get_logger().info("[HAND] Ready — finger tracking active.")

    # ── Sensor callbacks (called from SerialReader thread) ─────────────────────

    def _on_fsr(self, fields: dict):
        """Called by HandController for every FSR reading."""
        ch = int(fields.get("ch", -1))
        if ch < 0:
            return
        finger = FSR_FINGER.get(ch)
        if finger is None:
            return
        idx = _FINGER_IDX.get(finger)
        if idx is None:
            return
        with self._lock:
            if "pct"    in fields: self._fsr_pct[idx] = float(fields["pct"])
            if "raw"    in fields: self._fsr_raw[idx] = int(fields["raw"])

    def _on_ina(self, fields: dict):
        """Called by HandController for every INA219 reading."""
        ch = int(fields.get("ch", -1))
        if ch < 0:
            return
        finger = INA_FINGER.get(ch)
        if finger is None:
            return
        idx = _FINGER_IDX.get(finger)
        if idx is None:
            return
        with self._lock:
            if "current_ma" in fields: self._current_ma[idx] = float(fields["current_ma"])
            if "power_mw"   in fields: self._power_mw[idx]   = float(fields["power_mw"])

    # ── Publish ────────────────────────────────────────────────────────────────

    def _publish_all(self):
        with self._lock:
            fsr_pct    = list(self._fsr_pct)
            fsr_raw    = list(self._fsr_raw)
            current_ma = list(self._current_ma)
            power_mw   = list(self._power_mw)

        msg_fsr_pct = Float32MultiArray()
        msg_fsr_pct.data = fsr_pct
        self._pub_fsr_pct.publish(msg_fsr_pct)

        msg_fsr_raw = Int32MultiArray()
        msg_fsr_raw.data = fsr_raw
        self._pub_fsr_raw.publish(msg_fsr_raw)

        msg_current = Float32MultiArray()
        msg_current.data = current_ma
        self._pub_current_ma.publish(msg_current)

        msg_power = Float32MultiArray()
        msg_power.data = power_mw
        self._pub_power_mw.publish(msg_power)

    # ── Hand command subscriber ────────────────────────────────────────────────

    def _command_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'pause':
            self._paused = True
            self.get_logger().info('[HAND] Paused finger tracking.')
        elif cmd == 'resume':
            self._paused = False
            self.get_logger().info('[HAND] Resumed finger tracking.')
        elif cmd == 'estop':
            self._paused = True
            self._ctrl.estop()
            self.get_logger().info('[HAND] ESTOP — servos disabled.')
        elif cmd == 'resume_all':
            self._ctrl.resume_all()
            self._paused = False
            self.get_logger().info('[HAND] RESUME_ALL — servos re-enabled.')

    # ── Finger command subscriber ──────────────────────────────────────────────

    def _finger_cmd_cb(self, msg: Float32MultiArray):
        """
        Receive 5 values 0.0 (open) – 1.0 (closed) and send servo MOVE commands
        for the curve1 joint of each finger.
        """
        if self._paused:
            return
        if len(msg.data) < 5:
            self.get_logger().warn(
                f"/hand/finger_cmd expected 5 values, got {len(msg.data)}")
            return

        angle_range = self._closed_angle - self._open_angle
        for i, finger in enumerate(FINGER_ORDER):
            pct   = max(0.0, min(1.0, float(msg.data[i])))
            angle = self._open_angle + pct * angle_range
            if (self._last_angles[i] is not None and
                    abs(angle - self._last_angles[i]) < self._angle_deadband):
                continue
            self._last_angles[i] = angle
            try:
                self._ctrl.move_finger_curve1(finger, angle)
                if finger != "thumb":
                    self._ctrl.move_finger_curve2(finger, angle)
                self.get_logger().info(f"[HAND] MOVE {finger} → {angle:.1f}°")
            except Exception as e:
                self.get_logger().warn(f"[HAND] move_finger_curve1 {finger}: {e}")

    # ── Shutdown ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info("Shutting down HandNode — closing fingers...")
        try:
            closed_angle = self.get_parameter("closed_angle").value
            for finger in FINGER_ORDER:
                self._ctrl.move_finger_curve1(finger, closed_angle)
                if finger != "thumb":
                    self._ctrl.move_finger_curve2(finger, closed_angle)
        except Exception:
            pass
        import time
        time.sleep(2.0)
        self._ctrl.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
