#!/usr/bin/env python3
"""
glove_node.py — ROS2 node for the wearable glove over BLE (Nordic UART Service)

Publishes:
  /glove/finger_raw   (std_msgs/Int32MultiArray)  — raw ADC values 0-4095, 5 elements
  /glove/finger_pct   (std_msgs/Float32MultiArray) — mapped 0.0-1.0, 5 elements

Subscribes:
  /glove/servo_cmd    (std_msgs/Int32MultiArray)  — servo positions 0-1000, 5 elements
                                                    [thumb, index, middle, ring, pinky]

On first connect a two-step calibration runs automatically:
  1. Hold hand OPEN  for 3 s  → records open  (0 %) reference per finger
  2. Hold hand CLOSED for 3 s → records closed (100 %) reference per finger
"""

import asyncio
import json
import os
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
from std_msgs.msg import Bool

from bleak import BleakScanner, BleakClient

DEVICE_NAME = "wearable_glove"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]

# Fallback limits used before/if calibration is skipped
_DEFAULT_OPEN   = [2600, 2600, 2600, 2600, 2600]
_DEFAULT_CLOSED = [4000, 4000, 4000, 4000, 4000]

CALIB_HOLD_SECS   = 5.0   # seconds to hold each position
CALIB_COUNTDOWN   = 5     # countdown seconds before sampling starts

SERVO_SETTLE_SECS =10.0 # seconds to wait after commanding servos before reading pots

CALIB_FILE = os.path.expanduser("~/.ros/glove_calibration.json")


def _make_limits(open_vals, closed_vals):
    """Build per-finger [open_raw, closed_raw] from calibration samples."""
    limits = {}
    for i, name in enumerate(FINGER_NAMES):
        op = open_vals[i]
        cl = closed_vals[i]
        if abs(cl - op) < 50:
            cl = op + 50
        limits[name] = [op, cl]
    return limits


def map_to_pct(raw, finger, limits):
    """Map raw ADC to 0.0 (open) – 1.0 (closed), respecting sensor direction."""
    op, cl = limits[finger]
    lo = min(op, cl)
    hi = max(op, cl)
    raw = max(lo, min(raw, hi))
    if cl == op:
        return 0.0
    return max(0.0, min(1.0, (raw - op) / (cl - op)))


def parse_raw_line(line):
    parts = line.split(",")
    if len(parts) >= 7 and parts[0] == "RAW":
        try:
            return [int(parts[i]) for i in range(2, 7)]
        except ValueError:
            pass
    return None


def format_haptic(values):
    v = [max(0, min(1000, int(x))) for x in values]
    return f"A{v[0]}B{v[1]}C{v[2]}D{v[3]}E{v[4]}\n".encode()


class GloveNode(Node):
    def __init__(self):
        super().__init__("glove_node")

        self.declare_parameter("device_name",       DEVICE_NAME)
        self.declare_parameter("publish_rate_hz",   50.0)
        self.declare_parameter("heartbeat_rate_hz", 5.0)
        self.declare_parameter("skip_calibration",  False)

        self._device_name    = self.get_parameter("device_name").value
        self._pub_period     = 1.0 / self.get_parameter("publish_rate_hz").value
        self._hb_period      = 1.0 / self.get_parameter("heartbeat_rate_hz").value
        self._skip_calib     = self.get_parameter("skip_calibration").value

        self._pub_raw         = self.create_publisher(Int32MultiArray,   "/glove/finger_raw", 10)
        self._pub_pct         = self.create_publisher(Float32MultiArray, "/glove/finger_pct", 10)
        self._pub_arm_cmd     = self.create_publisher(String, "/arm_control_command", 10)
        self._pub_calibrating = self.create_publisher(Bool, "/glove/calibrating", 10)
        self._sub_servo = self.create_subscription(
            Int32MultiArray, "/glove/servo_cmd", self._servo_cmd_cb, 10)
        self._sub_cmd = self.create_subscription(
            String, "/glove/command", self._command_cb, 10)

        self._lock            = threading.Lock()
        self._raw_values      = [0] * 5
        self._servo_values    = [1000] * 5
        self._haptics_enabled = False   # set True via /glove/command 'haptics_on'
        self._rx_buf          = ""
        self._calibrated      = False
        self._limits          = _make_limits(_DEFAULT_OPEN, _DEFAULT_CLOSED)

        # Servo-pot calibration: pot readings at servo cmd 0 and 1000
        self._pot_at_cmd0    = None   # list of 5 ints, set after servo calib
        self._pot_at_cmd1000 = None   # list of 5 ints, set after servo calib
        self._servo_calib_done = False

        self._ble_client  = None
        self._stop_event  = asyncio.Event()
        self._ble_loop    = asyncio.new_event_loop()

        self._pub_timer = self.create_timer(self._pub_period, self._publish_fingers)

        self._ble_thread = threading.Thread(target=self._run_ble_thread, daemon=True)
        self._ble_thread.start()

        self.get_logger().info(f"GloveNode started — scanning for '{self._device_name}'")

    # ── Publishing ─────────────────────────────────────────────────────────────

    def _publish_fingers(self):
        with self._lock:
            if not self._calibrated:
                return
            raw    = list(self._raw_values)
            limits = dict(self._limits)
        pct = [map_to_pct(raw[i], FINGER_NAMES[i], limits) for i in range(5)]
        raw_msg = Int32MultiArray()
        raw_msg.data = raw
        self._pub_raw.publish(raw_msg)
        pct_msg = Float32MultiArray()
        pct_msg.data = pct
        self._pub_pct.publish(pct_msg)

    # ── Servo command ──────────────────────────────────────────────────────────

    def _servo_cmd_cb(self, msg):
        if len(msg.data) < 5:
            return
        with self._lock:
            if self._haptics_enabled:
                self._servo_values = [max(0, min(1000, int(v))) for v in msg.data[:5]]

    # ── Commands ───────────────────────────────────────────────────────────────

    def _command_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'lock':
            self.lock_at_current_position()
            self.get_logger().info('[GLOVE] Locked at current position.')
        elif cmd == 'free':
            with self._lock:
                self._servo_values = [1000] * 5
            self.get_logger().info('[GLOVE] Servos freed.')
        elif cmd == 'haptics_on':
            self._haptics_enabled = True
            self.get_logger().info('[GLOVE] Haptics enabled.')
        elif cmd == 'haptics_off':
            self._haptics_enabled = False
            with self._lock:
                self._servo_values = [1000] * 5
            self.get_logger().info('[GLOVE] Haptics disabled — servos freed.')

    # ── BLE thread ─────────────────────────────────────────────────────────────

    def _run_ble_thread(self):
        asyncio.set_event_loop(self._ble_loop)
        self._ble_loop.run_until_complete(self._ble_main())

    def _on_ble_disconnect(self, client):
        self.get_logger().warn("[BLE] Disconnected.")
        self._stop_event.set()

    def _on_notify(self, sender, data):
        self._rx_buf += data.decode("utf-8", errors="ignore")
        while "\n" in self._rx_buf:
            line, self._rx_buf = self._rx_buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            values = parse_raw_line(line)
            if values:
                with self._lock:
                    self._raw_values = values

    def _is_nus_ready(self, client):
        try:
            for svc in client.services:
                for char in svc.characteristics:
                    if char.uuid.lower() == NUS_RX_UUID.lower():
                        return True
        except Exception:
            pass
        return False

    async def _safe_write(self, client, data):
        for attempt in range(3):
            try:
                await client.write_gatt_char(NUS_RX_UUID, data, response=False)
                return True
            except Exception as e:
                if attempt < 2:
                    await asyncio.sleep(0.15)
                else:
                    self.get_logger().error(f"[BLE] Write failed: {e}")
                    return False
        return False

    # ── Calibration ────────────────────────────────────────────────────────────

    async def _collect_samples(self, duration_secs):
        """Collect raw finger samples for `duration_secs` and return averages."""
        samples = []
        deadline = self._ble_loop.time() + duration_secs
        while self._ble_loop.time() < deadline:
            with self._lock:
                samples.append(list(self._raw_values))
            await asyncio.sleep(0.05)  # 20 Hz sampling
        if not samples:
            return [0] * 5
        return [int(sum(s[i] for s in samples) / len(samples)) for i in range(5)]

    def _save_servo_calib(self):
        data = {"pot_at_cmd0": self._pot_at_cmd0, "pot_at_cmd1000": self._pot_at_cmd1000}
        try:
            os.makedirs(os.path.dirname(CALIB_FILE), exist_ok=True)
            with open(CALIB_FILE, "w") as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f"[CALIB] Servo calib saved to {CALIB_FILE}")
        except Exception as e:
            self.get_logger().warn(f"[CALIB] Could not save servo calib: {e}")

    def _load_servo_calib(self) -> bool:
        if not os.path.exists(CALIB_FILE):
            return False
        try:
            with open(CALIB_FILE) as f:
                data = json.load(f)
            with self._lock:
                self._pot_at_cmd0      = data["pot_at_cmd0"]
                self._pot_at_cmd1000   = data["pot_at_cmd1000"]
                self._servo_calib_done = True
            self.get_logger().info(f"[CALIB] Servo calib loaded from {CALIB_FILE}")
            return True
        except Exception as e:
            self.get_logger().warn(f"[CALIB] Could not load servo calib: {e}")
            return False

    def _cprint(self, msg=""):
        print(msg, flush=True)

    def _arm_pause(self):
        msg = String(); msg.data = 'pause'
        self._pub_arm_cmd.publish(msg)
        cal = Bool(); cal.data = True
        self._pub_calibrating.publish(cal)

    def _arm_resume(self):
        msg = String(); msg.data = 'resume'
        self._pub_arm_cmd.publish(msg)
        cal = Bool(); cal.data = False
        self._pub_calibrating.publish(cal)

    async def _run_calibration(self):
        """Servo-pot sweep (once, skipped if saved), then open/closed finger calibration (always)."""
        self._arm_pause()
        self.get_logger().info("[CALIB] Arm paused for calibration.")

        with self._lock:
            self._servo_values = [1000] * 5

        self._cprint("\n" + "="*50)
        self._cprint("  GLOVE CALIBRATION")
        self._cprint("="*50)

        # Servo-pot sweep — skip if already loaded from file
        if not self._servo_calib_done:
            await self._run_servo_calibration()
        else:
            self._cprint("\n  [Servo-pot calibration loaded from file — skipping sweep]")

        # Servos free before finger calibration
        with self._lock:
            self._servo_values = [1000] * 5

        # ── Step 1: open hand ──────────────────────────────────────────────────
        self._cprint(f"\n  Step 1/2 — OPEN your hand fully")
        for i in range(CALIB_COUNTDOWN, 0, -1):
            self._cprint(f"  Sampling in {i}...")
            await asyncio.sleep(1.0)
        self._cprint("  Sampling OPEN position...")
        open_vals = await self._collect_samples(CALIB_HOLD_SECS)
        self._cprint(f"  Open  raw: {open_vals}")

        # ── Step 2: closed hand ────────────────────────────────────────────────
        self._cprint(f"\n  Step 2/2 — CLOSE your hand fully (make a fist)")
        for i in range(CALIB_COUNTDOWN, 0, -1):
            self._cprint(f"  Sampling in {i}...")
            await asyncio.sleep(1.0)
        self._cprint("  Sampling CLOSED position...")
        closed_vals = await self._collect_samples(CALIB_HOLD_SECS)
        self._cprint(f"  Closed raw: {closed_vals}")

        # ── Apply finger limits ────────────────────────────────────────────────
        new_limits = _make_limits(open_vals, closed_vals)
        with self._lock:
            self._limits     = new_limits
            self._calibrated = True

        self._cprint("\n  Finger calibration complete:")
        for name in FINGER_NAMES:
            op, cl = new_limits[name]
            self._cprint(f"    {name:8s}  open={op}  closed={cl}")
        self._cprint("="*50 + "\n")
        self.get_logger().info("[CALIB] Finger calibration applied.")
        self._arm_resume()
        self.get_logger().info("[CALIB] Arm resumed.")

    # ── Servo-pot calibration & lock ──────────────────────────────────────────

    async def _run_servo_calibration(self):
        """Sweep all servos to 0 then 1000, record pot readings at each extreme."""
        self._cprint("\n  [Servo-pot calibration — automated, do not move fingers]")

        # Command all servos to 0 (locked/taut), wait for settle
        with self._lock:
            self._servo_values = [0] * 5
        self._cprint("  Servos → 0, settling...")
        await asyncio.sleep(SERVO_SETTLE_SECS)
        pot_at_0 = await self._collect_samples(1.0)
        self._cprint(f"  pot@0   : {pot_at_0}")

        # Command all servos to 1000 (free), wait for settle
        with self._lock:
            self._servo_values = [1000] * 5
        self._cprint("  Servos → 1000, settling...")
        await asyncio.sleep(SERVO_SETTLE_SECS)
        pot_at_1000 = await self._collect_samples(1.0)
        self._cprint(f"  pot@1000: {pot_at_1000}")

        with self._lock:
            self._pot_at_cmd0    = pot_at_0
            self._pot_at_cmd1000 = pot_at_1000
            self._servo_calib_done = True

        self._cprint("  Servo-pot calibration done.\n")
        self._save_servo_calib()

    def lock_at_current_position(self):
        """
        Command each servo to the position that matches the current pot reading.
        Holds the spool at the finger's current curl without adding tension.
        """
        with self._lock:
            if not self._servo_calib_done:
                self.get_logger().warn("[GLOVE] Lock ignored — servo calibration not done yet.")
                return
            raw    = list(self._raw_values)
            at0    = list(self._pot_at_cmd0)
            at1000 = list(self._pot_at_cmd1000)

        cmds = []
        for i in range(5):
            span = at1000[i] - at0[i]
            if abs(span) < 20:
                self.get_logger().warn(f"[GLOVE] Finger {i} ({FINGER_NAMES[i]}): pot span too small ({span}), skipping lock.")
                cmds.append(self._servo_values[i])
                continue
            cmd = int((raw[i] - at0[i]) / span * 1000)
            cmd = max(0, min(1000, cmd))
            cmds.append(cmd)
            self.get_logger().info(
                f"[GLOVE] Lock {FINGER_NAMES[i]}: pot={raw[i]}  at0={at0[i]}  at1000={at1000[i]}  → cmd={cmd}")

        with self._lock:
            self._servo_values = cmds

    # ── Heartbeat / main BLE loop ──────────────────────────────────────────────

    async def _heartbeat_loop(self, client):
        while not self._stop_event.is_set():
            with self._lock:
                values = list(self._servo_values)
            ok = await self._safe_write(client, format_haptic(values))
            if not ok:
                self._stop_event.set()
                break
            await asyncio.sleep(self._hb_period)

    def _reset_ble_adapter(self):
        """Reset the Bluetooth adapter to clear stale BlueZ state."""
        try:
            subprocess.run(["hciconfig", "hci0", "reset"],
                           timeout=5, capture_output=True)
            self.get_logger().info("[BLE] Adapter reset.")
        except Exception as e:
            self.get_logger().warn(f"[BLE] Adapter reset failed: {e}")

    async def _ble_main(self):
        first_connect = True
        scan_failures = 0
        while rclpy.ok():
            self._stop_event.clear()
            self.get_logger().info(f"[BLE] Scanning for '{self._device_name}'...")
            device = await BleakScanner.find_device_by_name(self._device_name, timeout=30.0)
            if device is None:
                scan_failures += 1
                self.get_logger().warn(
                    f"[BLE] Device not found (attempt {scan_failures}). Retrying in 3s...")
                if scan_failures % 3 == 0:
                    self.get_logger().warn("[BLE] Resetting Bluetooth adapter...")
                    await asyncio.get_event_loop().run_in_executor(
                        None, self._reset_ble_adapter)
                    await asyncio.sleep(2.0)
                else:
                    await asyncio.sleep(3.0)
                continue
            scan_failures = 0
            self.get_logger().info(f"[BLE] Found: {device.name}  [{device.address}]")
            # Brief pause so BlueZ can release any stale notify from a prior session
            await asyncio.sleep(1.0)
            try:
                async with BleakClient(device, disconnected_callback=self._on_ble_disconnect) as client:
                    deadline = self._ble_loop.time() + 10.0
                    while self._ble_loop.time() < deadline:
                        if self._is_nus_ready(client):
                            break
                        await asyncio.sleep(0.2)
                    else:
                        self.get_logger().error("[BLE] Service discovery timed out.")
                        continue

                    self._ble_client = client
                    self.get_logger().info("[BLE] Connected and ready.")
                    # Try to stop any stale notify before subscribing
                    try:
                        await client.stop_notify(NUS_TX_UUID)
                    except Exception:
                        pass
                    await asyncio.sleep(0.3)
                    await client.start_notify(NUS_TX_UUID, self._on_notify)

                    # Load servo calib from file if available (once-only calibration).
                    # Finger open/close calibration always runs unless skip_calibration=true.
                    if first_connect:
                        first_connect = False
                        self._load_servo_calib()
                        if not self._skip_calib:
                            hb_task    = asyncio.create_task(self._heartbeat_loop(client))
                            calib_task = asyncio.create_task(self._run_calibration())
                            await calib_task
                            await hb_task
                        else:
                            await self._heartbeat_loop(client)
                    else:
                        await self._heartbeat_loop(client)
                    try:
                        await client.stop_notify(NUS_TX_UUID)
                    except Exception:
                        pass
            except Exception as e:
                self.get_logger().error(f"[BLE] Connection error: {e}")
                if "NotPermitted" in str(e) or "Notify acquired" in str(e):
                    self.get_logger().warn("[BLE] Stale notify detected — resetting adapter...")
                    await asyncio.get_event_loop().run_in_executor(
                        None, self._reset_ble_adapter)
                    await asyncio.sleep(2.0)
            self._ble_client = None
            self.get_logger().info("[BLE] Reconnecting in 3s...")
            await asyncio.sleep(3.0)

    def destroy_node(self):
        self.get_logger().info("Shutting down GloveNode...")
        self._stop_event.set()
        self._ble_loop.call_soon_threadsafe(self._ble_loop.stop)
        self._ble_thread.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GloveNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
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
