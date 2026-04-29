#!/usr/bin/env python3
"""
glove_node.py — ROS2 node for the wearable glove over BLE (Nordic UART Service)

Publishes:
  /glove/finger_raw   (std_msgs/Int32MultiArray)  — raw ADC values 0-4095, 5 elements
  /glove/finger_pct   (std_msgs/Float32MultiArray) — mapped 0.0-1.0, 5 elements

Subscribes:
  /glove/servo_cmd    (std_msgs/Int32MultiArray)  — servo positions 0-1000, 5 elements
                                                    [thumb, index, middle, ring, pinky]
  /glove/command      (std_msgs/String)           — 'lock' | 'free' | 'haptics_on' | 'haptics_off'

Lock behaviour:
  Finger at 0% (open)    → servo cmd 1000 (free)
  Finger at 75% (curled) → servo cmd  250
  Finger at 100%(closed) → servo cmd    0 (taut)
"""

import asyncio
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String

from bleak import BleakScanner, BleakClient

DEVICE_NAME = "wearable_glove"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]

# ── Per-finger limits [open_adc, closed_adc] ─────────────────────────────────
# Tune these to match the physical glove's potentiometer range.
#   open_adc   = raw ADC when finger is fully extended (string slack)
#   closed_adc = raw ADC when finger is fully curled   (string taut)
FINGER_LIMITS = {
    "thumb":  [2100, 4095],
    "index":  [1100, 3700],
    "middle": [2300, 4095],
    "ring":   [1900, 4095],
    "pinky":  [2050, 4095],
}


def map_to_pct(raw: int, finger: str) -> float:
    """Map raw ADC to 0.0 (open) – 1.0 (closed) using FINGER_LIMITS."""
    lo, hi = FINGER_LIMITS[finger]
    if lo > hi:
        lo, hi = hi, lo
    raw = max(lo, min(raw, hi))
    return (raw - lo) / (hi - lo) if hi != lo else 0.0


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

        self._device_name = self.get_parameter("device_name").value
        self._pub_period  = 1.0 / self.get_parameter("publish_rate_hz").value
        self._hb_period   = 1.0 / self.get_parameter("heartbeat_rate_hz").value

        self._pub_raw = self.create_publisher(Int32MultiArray,   "/glove/finger_raw", 10)
        self._pub_pct = self.create_publisher(Float32MultiArray, "/glove/finger_pct", 10)
        self._sub_servo = self.create_subscription(
            Int32MultiArray, "/glove/servo_cmd", self._servo_cmd_cb, 10)
        self._sub_cmd = self.create_subscription(
            String, "/glove/command", self._command_cb, 10)

        self._lock            = threading.Lock()
        self._raw_values      = [0] * 5
        self._servo_values    = [1000] * 5
        self._haptics_enabled = False   # toggled via /glove/command 'haptics_on/off'
        self._rx_buf          = ""

        self._ble_client = None
        self._stop_event = asyncio.Event()
        self._ble_loop   = asyncio.new_event_loop()

        self._pub_timer = self.create_timer(self._pub_period, self._publish_fingers)

        self._ble_thread = threading.Thread(target=self._run_ble_thread, daemon=True)
        self._ble_thread.start()

        self.get_logger().info(f"GloveNode started — scanning for '{self._device_name}'")

    # ── Publishing ─────────────────────────────────────────────────────────────

    def _publish_fingers(self):
        with self._lock:
            raw = list(self._raw_values)
        pct = [map_to_pct(raw[i], FINGER_NAMES[i]) for i in range(5)]
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

    # ── Lock ───────────────────────────────────────────────────────────────────

    def lock_at_current_position(self):
        """
        Command each servo to the position matching the finger's current curl.
        Uses FINGER_LIMITS to map raw ADC → 0.0–1.0, then inverts to servo cmd:
          0%  (open)   → cmd 1000 (free)
          75% (curled) → cmd  250
          100%(closed) → cmd    0 (taut)
        """
        with self._lock:
            raw = list(self._raw_values)

        cmds = []
        for i, name in enumerate(FINGER_NAMES):
            pct = map_to_pct(raw[i], name)
            cmd = max(0, min(1000, int((1.0 - pct) * 1000)))
            cmds.append(cmd)
            self.get_logger().info(
                f"[GLOVE] Lock {name}: raw={raw[i]}  pct={pct:.2f}  → cmd={cmd}")

        with self._lock:
            self._servo_values = cmds

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
                    try:
                        await client.stop_notify(NUS_TX_UUID)
                    except Exception:
                        pass
                    await asyncio.sleep(0.3)
                    await client.start_notify(NUS_TX_UUID, self._on_notify)
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

        # Free all servos while the BLE connection is still open.
        # Schedule the write on the BLE loop from this (ROS) thread and wait
        # for it to complete before tearing down the loop.
        with self._lock:
            self._servo_values = [1000] * 5
        if self._ble_client is not None:
            future = asyncio.run_coroutine_threadsafe(
                self._safe_write(self._ble_client, format_haptic([1000] * 5)),
                self._ble_loop,
            )
            try:
                future.result(timeout=1.0)
            except Exception:
                pass

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
