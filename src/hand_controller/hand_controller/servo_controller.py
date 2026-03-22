#!/usr/bin/env python3
"""
servo_controller.py — Python driver for servo_controller.ino
Teensy 4.1 + PCA9685 · 15x MG90S servos

USAGE (standalone CLI):
    python3 servo_controller.py --port /dev/ttyACM0
    python3 servo_controller.py --port /dev/ttyACM0 --baud 115200

ROS INTEGRATION:
    Import ServoController and wrap it in a thin ROS node — see the
    RosServoNode stub at the bottom of this file.  The class itself has
    no ROS dependency; just replace the CLI loop with your subscriber
    callbacks and publisher calls.

SERIAL PROTOCOL (matches servo_controller.ino):
    Commands  →  Teensy  :  "MOVE 2 90\\n"
    Responses ←  Teensy  :  "MOVE_DONE ch=2 angle=90.0\\n"
"""

import argparse
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial not found — run:  pip install pyserial")


# ─────────────────────────────────────────────────────────────────────────────
# Data model
# ─────────────────────────────────────────────────────────────────────────────

NUM_CHANNELS = 15

@dataclass
class ChannelState:
    channel:        int
    angle:          float = 90.0    # current (reported) angle
    target:         float = 90.0    # in-flight target
    commanded:      float = 90.0    # last explicitly commanded angle
    enabled:        bool  = True
    stopped:        bool  = False
    joint_min:      float = 0.0
    joint_max:      float = 180.0
    init_angle:     float = 90.0


# ─────────────────────────────────────────────────────────────────────────────
# Core controller class  (no ROS dependency)
# ─────────────────────────────────────────────────────────────────────────────

class ServoController:
    """
    Manages one serial connection to a Teensy running servo_controller.ino.

    All public methods are thread-safe.  A background reader thread feeds
    incoming lines to _handle_line(), which updates self.channels and fires
    any registered callbacks.

    Callbacks
    ---------
    Register with on_event(tag, fn).  fn receives a dict of parsed fields.
    Tag is the first word of the Teensy response: READY, MOVE_DONE, ESTOP, etc.
    Use tag="*" to receive every event.

    Example
    -------
    >>> ctrl = ServoController("/dev/ttyACM0")
    >>> ctrl.wait_ready(timeout=10)
    >>> ctrl.move(0, 45.0)
    >>> ctrl.wait_move_done(0, timeout=5)
    """

    def __init__(self, port: str, baud: int = 115200, connect: bool = True):
        self.port  = port
        self.baud  = baud
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()

        # Per-channel state mirrored from Teensy
        self.channels: Dict[int, ChannelState] = {
            ch: ChannelState(channel=ch) for ch in range(NUM_CHANNELS)
        }
        self.estop_active   = False
        self.teensy_ready   = False
        self.speed_dps      = 60.0

        # Event callbacks  {tag: [fn, ...]}
        self._callbacks: Dict[str, List[Callable]] = {}
        # Move-done events keyed by channel
        self._move_done_events: Dict[int, threading.Event] = {
            ch: threading.Event() for ch in range(NUM_CHANNELS)
        }
        self._ready_event = threading.Event()

        self._reader_thread: Optional[threading.Thread] = None
        self._stop_reader   = threading.Event()

        if connect:
            self.connect()

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self._stop_reader.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="servo-reader"
        )
        self._reader_thread.start()
        print(f"[serial] connected to {self.port} @ {self.baud}")

    def disconnect(self):
        self._stop_reader.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=2)
        if self._ser and self._ser.is_open:
            self._ser.close()
        print("[serial] disconnected")

    def wait_ready(self, timeout: float = 15.0) -> bool:
        """Block until Teensy sends READY (fires after boot homing sequence)."""
        return self._ready_event.wait(timeout=timeout)

    # ── Commands ──────────────────────────────────────────────────────────────

    def move(self, channel: int, angle: float):
        """Command one channel to move gradually to angle (degrees)."""
        self._validate_channel(channel)
        ch = self.channels[channel]
        angle = max(ch.joint_min, min(ch.joint_max, float(angle)))
        self._move_done_events[channel].clear()
        self._send(f"MOVE {channel} {angle:.1f}")

    def stop(self, channel: int):
        """Immediately disable one channel (servo goes limp)."""
        self._validate_channel(channel)
        self._send(f"STOP {channel}")

    def estop(self):
        """Emergency stop — disable all channels immediately."""
        self._send("ESTOP")

    def resume(self, channel: int):
        """Re-enable one stopped channel; it continues toward last commanded angle."""
        self._validate_channel(channel)
        self._send(f"RESUME {channel}")

    def resume_all(self):
        """Clear E-STOP and re-enable all channels."""
        self._send("RESUME_ALL")

    def set_speed(self, dps: float):
        """Set global slew rate in degrees/second (5–720)."""
        self._send(f"SPEED {dps:.1f}")

    def request_status(self):
        """Ask Teensy to send STATUS lines for all channels."""
        self._send("STATUS")

    def wait_move_done(self, channel: int, timeout: float = 30.0) -> bool:
        """Block until MOVE_DONE is received for this channel."""
        return self._move_done_events[channel].wait(timeout=timeout)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def on_event(self, tag: str, fn: Callable):
        """
        Register a callback for a specific response tag (or "*" for all).
        fn(fields: dict) is called from the reader thread.
        """
        tag = tag.upper()
        self._callbacks.setdefault(tag, []).append(fn)

    # ── Internal ──────────────────────────────────────────────────────────────

    def _send(self, cmd: str):
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.write((cmd + "\n").encode())

    def _validate_channel(self, ch: int):
        if ch < 0 or ch >= NUM_CHANNELS:
            raise ValueError(f"Channel {ch} out of range (0–{NUM_CHANNELS-1})")

    def _reader_loop(self):
        while not self._stop_reader.is_set():
            try:
                raw = self._ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if line:
                self._handle_line(line)

    def _handle_line(self, line: str):
        parts = line.split()
        if not parts:
            return
        tag = parts[0].upper()
        fields = _parse_fields(parts[1:])   # remaining "key=value" tokens

        # ── State updates ──────────────────────────────────────────────────

        if tag == "READY":
            self.teensy_ready = True
            self._ready_event.set()

        elif tag == "MOVE_START":
            ch = int(fields.get("ch", -1))
            if 0 <= ch < NUM_CHANNELS:
                self.channels[ch].target    = float(fields.get("to", 0))
                self.channels[ch].commanded = float(fields.get("to", 0))
                self.channels[ch].stopped   = False
                self.channels[ch].enabled   = True

        elif tag == "MOVE_DONE":
            ch = int(fields.get("ch", -1))
            if 0 <= ch < NUM_CHANNELS:
                self.channels[ch].angle  = float(fields.get("angle", 0))
                self.channels[ch].target = self.channels[ch].angle
                self._move_done_events[ch].set()

        elif tag == "STOP":
            ch = int(fields.get("ch", -1))
            if 0 <= ch < NUM_CHANNELS:
                self.channels[ch].angle   = float(fields.get("angle", 0))
                self.channels[ch].stopped = True
                self.channels[ch].enabled = False

        elif tag == "ESTOP":
            self.estop_active = True
            # "angle=A0,A1,...,A14"
            raw_angles = fields.get("angle", "")
            angles = raw_angles.split(",")
            for ch, a in enumerate(angles[:NUM_CHANNELS]):
                try:
                    self.channels[ch].angle   = float(a)
                    self.channels[ch].stopped = True
                    self.channels[ch].enabled = False
                except ValueError:
                    pass

        elif tag == "RESUME":
            ch = int(fields.get("ch", -1))
            if 0 <= ch < NUM_CHANNELS:
                self.channels[ch].stopped   = False
                self.channels[ch].enabled   = True
                self.channels[ch].target    = float(fields.get("target", self.channels[ch].angle))
                self._move_done_events[ch].clear()

        elif tag == "RESUME_ALL":
            self.estop_active = False
            for ch in range(NUM_CHANNELS):
                self.channels[ch].stopped = False
                self.channels[ch].enabled = True

        elif tag == "SPEED":
            self.speed_dps = float(fields.get("dps", self.speed_dps))

        elif tag == "STATUS":
            ch = int(fields.get("ch", -1))
            if 0 <= ch < NUM_CHANNELS:
                c = self.channels[ch]
                c.angle     = float(fields.get("angle",     c.angle))
                c.target    = float(fields.get("target",    c.target))
                c.commanded = float(fields.get("commanded", c.commanded))
                c.enabled   = bool(int(fields.get("enabled", int(c.enabled))))
                c.stopped   = bool(int(fields.get("stopped", int(c.stopped))))

        elif tag == "INFO":
            # Parse init table lines:  INFO init ch=N angle=A min=M max=X
            if parts[1:2] == ["init"]:
                ch = int(fields.get("ch", -1))
                if 0 <= ch < NUM_CHANNELS:
                    c = self.channels[ch]
                    c.init_angle = float(fields.get("angle", c.init_angle))
                    c.joint_min  = float(fields.get("min",   c.joint_min))
                    c.joint_max  = float(fields.get("max",   c.joint_max))
                    c.angle      = c.init_angle
                    c.target     = c.init_angle
                    c.commanded  = c.init_angle

        # ── Fire callbacks ─────────────────────────────────────────────────

        fields["_tag"]  = tag
        fields["_line"] = line
        for fn in self._callbacks.get(tag, []):
            fn(fields)
        for fn in self._callbacks.get("*", []):
            fn(fields)


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _parse_fields(tokens: List[str]) -> Dict[str, str]:
    """
    Convert ["ch=2", "angle=90.0", "enabled=1"] → {"ch": "2", "angle": "90.0", ...}
    Tokens without '=' are stored as {"token": "token"}.
    """
    out = {}
    for tok in tokens:
        if "=" in tok:
            k, _, v = tok.partition("=")
            out[k] = v
        else:
            out[tok] = tok
    return out


def list_ports() -> List[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


# ─────────────────────────────────────────────────────────────────────────────
# Interactive CLI
# ─────────────────────────────────────────────────────────────────────────────

HELP_TEXT = """
Commands
────────
  move <ch> <angle>    Move channel to angle (clamped to joint limits)
  stop <ch>            Disable one channel
  estop                Emergency stop all channels
  resume <ch>          Re-enable one stopped channel
  resume_all           Clear E-STOP, re-enable all
  speed <dps>          Set slew rate (degrees/second)
  status               Print mirrored state of all channels
  wait <ch>            Block until channel move completes
  help                 Show this message
  quit / exit          Disconnect and exit
"""

def run_cli(ctrl: ServoController):
    # Echo all Teensy output to stdout
    def _print_event(fields):
        tag  = fields.get("_tag", "")
        line = fields.get("_line", "")
        # ERR always shown in red-ish prefix; INFO shown as-is; others too
        prefix = ""
        if tag == "ERR":
            prefix = "[ERR] "
        elif tag == "INFO":
            prefix = "[INFO] "
        elif tag in ("MOVE_DONE", "MOVE_START"):
            prefix = "[MOVE] "
        elif tag in ("ESTOP", "STOP"):
            prefix = "[STOP] "
        elif tag in ("RESUME", "RESUME_ALL"):
            prefix = "[RESUME] "
        else:
            prefix = f"[{tag}] " if tag not in ("STATUS",) else ""
        print(f"\r{prefix}{line}")

    ctrl.on_event("*", _print_event)

    print("Waiting for Teensy to boot and home servos...")
    if not ctrl.wait_ready(timeout=20):
        print("Timed out waiting for READY — check port and wiring.")
        return

    print(HELP_TEXT)

    while True:
        try:
            raw = input("servo> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not raw:
            continue

        parts = raw.split()
        cmd   = parts[0].lower()

        if cmd in ("quit", "exit"):
            break

        elif cmd == "help":
            print(HELP_TEXT)

        elif cmd == "move":
            if len(parts) < 3:
                print("Usage: move <channel> <angle>")
                continue
            try:
                ctrl.move(int(parts[1]), float(parts[2]))
            except ValueError as e:
                print(f"Error: {e}")

        elif cmd == "stop":
            if len(parts) < 2:
                print("Usage: stop <channel>")
                continue
            try:
                ctrl.stop(int(parts[1]))
            except ValueError as e:
                print(f"Error: {e}")

        elif cmd == "estop":
            ctrl.estop()

        elif cmd == "resume":
            if len(parts) < 2:
                print("Usage: resume <channel>")
                continue
            try:
                ctrl.resume(int(parts[1]))
            except ValueError as e:
                print(f"Error: {e}")

        elif cmd == "resume_all":
            ctrl.resume_all()

        elif cmd == "speed":
            if len(parts) < 2:
                print("Usage: speed <dps>")
                continue
            try:
                ctrl.set_speed(float(parts[1]))
            except ValueError as e:
                print(f"Error: {e}")

        elif cmd == "status":
            print(f"\n{'CH':<4} {'ANGLE':>7} {'TARGET':>7} {'CMD':>7} "
                  f"{'MIN':>6} {'MAX':>6} {'EN':>3} {'STOP':>4}")
            print("─" * 55)
            for ch, c in ctrl.channels.items():
                print(f"{ch:<4} {c.angle:>7.1f} {c.target:>7.1f} {c.commanded:>7.1f}"
                      f" {c.joint_min:>6.1f} {c.joint_max:>6.1f}"
                      f" {'Y' if c.enabled else 'N':>3}"
                      f" {'Y' if c.stopped else 'N':>4}")
            print()
            ctrl.request_status()     # also refresh from Teensy

        elif cmd == "wait":
            if len(parts) < 2:
                print("Usage: wait <channel>")
                continue
            try:
                ch = int(parts[1])
                print(f"Waiting for channel {ch} to finish moving...")
                done = ctrl.wait_move_done(ch, timeout=30)
                print("Done." if done else "Timed out.")
            except ValueError as e:
                print(f"Error: {e}")

        else:
            print(f"Unknown command '{cmd}' — type 'help'")


# ─────────────────────────────────────────────────────────────────────────────
# ROS node stub  (replace CLI loop with this when integrating)
# ─────────────────────────────────────────────────────────────────────────────
#
# To convert to a ROS 2 node:
#
#   1. pip install rclpy  (in your ROS workspace)
#   2. Uncomment the block below and fill in your topic names.
#   3. Replace run_cli(ctrl) with run_ros_node(ctrl) in __main__.
#
# The ServoController class is unchanged — ROS is just a thin shell around it.
#
# ─────────────────────────────────────────────────────────────────────────────
#
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray, Bool, Int32MultiArray
#
# class RosServoNode(Node):
#     def __init__(self, ctrl: ServoController):
#         super().__init__("servo_controller")
#         self.ctrl = ctrl
#
#         # Subscribe: receive [channel, angle] pairs
#         self.create_subscription(
#             Float32MultiArray, "/servo/move",
#             self._cb_move, 10
#         )
#         # Subscribe: emergency stop flag
#         self.create_subscription(
#             Bool, "/servo/estop",
#             self._cb_estop, 10
#         )
#         # Publish: current angles for all channels at 10 Hz
#         self._pub_angles = self.create_publisher(Float32MultiArray, "/servo/angles", 10)
#         self.create_timer(0.1, self._pub_state)
#
#         # Forward Teensy ERR lines to ROS logger
#         ctrl.on_event("ERR", lambda f: self.get_logger().error(f["_line"]))
#         ctrl.on_event("INFO", lambda f: self.get_logger().info(f["_line"]))
#
#     def _cb_move(self, msg: Float32MultiArray):
#         data = msg.data
#         for i in range(0, len(data) - 1, 2):
#             ch  = int(data[i])
#             ang = float(data[i + 1])
#             self.ctrl.move(ch, ang)
#
#     def _cb_estop(self, msg: Bool):
#         if msg.data:
#             self.ctrl.estop()
#         else:
#             self.ctrl.resume_all()
#
#     def _pub_state(self):
#         angles = [self.ctrl.channels[ch].angle for ch in range(NUM_CHANNELS)]
#         self._pub_angles.publish(Float32MultiArray(data=angles))
#
#
# def run_ros_node(ctrl: ServoController):
#     rclpy.init()
#     node = RosServoNode(ctrl)
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Servo controller CLI / ROS bridge")
    parser.add_argument("--port",  default=None,  help="Serial port (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--baud",  default=115200, type=int, help="Baud rate (default 115200)")
    parser.add_argument("--list",  action="store_true", help="List available serial ports and exit")
    args = parser.parse_args()

    if args.list:
        ports = list_ports()
        if ports:
            print("Available ports:")
            for p in ports:
                print(f"  {p}")
        else:
            print("No serial ports found.")
        return

    if args.port is None:
        ports = list_ports()
        if not ports:
            sys.exit("No serial ports found — specify one with --port")
        args.port = ports[0]
        print(f"No port specified — auto-selecting {args.port}")

    ctrl = ServoController(args.port, args.baud)

    try:
        run_cli(ctrl)
        # For ROS: replace run_cli(ctrl) with run_ros_node(ctrl)
    finally:
        ctrl.disconnect()


if __name__ == "__main__":
    main()
