#!/usr/bin/env python3
"""
glove_node.py — ROS2 node for the wearable glove over BLE (Nordic UART Service)

Publishes:
  /glove/finger_raw   (std_msgs/Int32MultiArray)  — raw ADC values 0-4095, 5 elements
  /glove/finger_pct   (std_msgs/Float32MultiArray) — mapped 0.0-1.0, 5 elements

Subscribes:
  /glove/servo_cmd    (std_msgs/Int32MultiArray)  — servo positions 0-1000, 5 elements
                                                    [thumb, index, middle, ring, pinky]
"""

import asyncio
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32MultiArray, Float32MultiArray

from bleak import BleakScanner, BleakClient

DEVICE_NAME = "wearable_glove"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]

FINGER_LIMITS = {
    "thumb":  [2600, 4000],
    "index":  [2600, 4000],
    "middle": [2600, 4000],
    "ring":   [2600, 4000],
    "pinky":  [2600, 4000],
}


def map_to_pct(raw, finger):
    lo, hi = FINGER_LIMITS[finger]
    if lo > hi:
        lo, hi = hi, lo
    raw = max(lo, min(raw, hi))
    if hi == lo:
        return 0.0
    return (raw - lo) / (hi - lo)


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

        self.declare_parameter("device_name", DEVICE_NAME)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("heartbeat_rate_hz", 5.0)

        self._device_name = self.get_parameter("device_name").value
        self._pub_period  = 1.0 / self.get_parameter("publish_rate_hz").value
        self._hb_period   = 1.0 / self.get_parameter("heartbeat_rate_hz").value

        self._pub_raw = self.create_publisher(Int32MultiArray, "/glove/finger_raw", 10)
        self._pub_pct = self.create_publisher(Float32MultiArray, "/glove/finger_pct", 10)
        self._sub_servo = self.create_subscription(
            Int32MultiArray, "/glove/servo_cmd", self._servo_cmd_cb, 10)

        self._lock         = threading.Lock()
        self._raw_values   = [0] * 5
        self._servo_values = [1000] * 5
        self._servo_dirty  = False
        self._rx_buf       = ""

        self._ble_client = None
        self._stop_event = asyncio.Event()
        self._ble_loop   = asyncio.new_event_loop()

        self._pub_timer = self.create_timer(self._pub_period, self._publish_fingers)

        self._ble_thread = threading.Thread(target=self._run_ble_thread, daemon=True)
        self._ble_thread.start()

        self.get_logger().info(f"GloveNode started — scanning for '{self._device_name}'")

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

    def _servo_cmd_cb(self, msg):
        if len(msg.data) < 5:
            self.get_logger().warn(f"/glove/servo_cmd expected 5 values, got {len(msg.data)}")
            return
        with self._lock:
            self._servo_values = [max(0, min(1000, int(v))) for v in msg.data[:5]]
            self._servo_dirty = True

    def _run_ble_thread(self):
        asyncio.set_event_loop(self._ble_loop)
        self._ble_loop.run_until_complete(self._ble_main())

    def _on_ble_disconnect(self, client):
        self.get_logger().warn("[BLE] Disconnected from ESP32.")
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
                    self.get_logger().error(f"[BLE] Write failed after 3 attempts: {e}")
                    return False
        return False

    async def _heartbeat_loop(self, client):
        while not self._stop_event.is_set():
            with self._lock:
                values = list(self._servo_values)
                self._servo_dirty = False
            ok = await self._safe_write(client, format_haptic(values))
            if not ok:
                self._stop_event.set()
                break
            await asyncio.sleep(self._hb_period)

    async def _ble_main(self):
        while rclpy.ok():
            self._stop_event.clear()
            self.get_logger().info(f"[BLE] Scanning for '{self._device_name}'...")
            device = await BleakScanner.find_device_by_name(self._device_name, timeout=10.0)
            if device is None:
                self.get_logger().warn("[BLE] Device not found. Retrying in 5s...")
                await asyncio.sleep(5.0)
                continue
            self.get_logger().info(f"[BLE] Found: {device.name}  [{device.address}]")
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
                    await client.start_notify(NUS_TX_UUID, self._on_notify)
                    await self._heartbeat_loop(client)
                    try:
                        await client.stop_notify(NUS_TX_UUID)
                    except Exception:
                        pass
            except Exception as e:
                self.get_logger().error(f"[BLE] Connection error: {e}")
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
