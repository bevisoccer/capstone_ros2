#!/usr/bin/env python3
import asyncio
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hand_interfaces.msg import RobotArmTarget
from hand_control.MotorController import MoteusMotorController

MOTOR_IDS = [1, 2, 3, 4]
ANGLE_LIMITS = {1: (-180, 180), 2: (-90, 30), 3: (-80, 80), 4: (-120, 120)}
MOTOR_TIMEOUTS = {1: 3.0, 2: 1.0, 3: 1.0, 4: 1.0}

PARK_VELOCITY_LIMIT    = 0.25
PARK_ACCEL_LIMIT       = 0.15
PARK_TORQUE_LIMIT      = 6.0
PARK_TIMEOUT_PER_MOTOR = 5.0

# Absolute raw positions — independent of zero offset drift
PARK_POSE_RAW   = {1: -0.474, 2: -0.486, 3: -0.255, 4:  0.239}
ORIGIN_POSE_RAW = {1: -0.218, 2: -0.351, 3: -0.515, 4: -0.061}

REF_X, REF_Y, REF_Z             = 0.20, 0.0, 0.24
REF_M1, REF_M2, REF_M3, REF_M4 = 96.9, -45.0, 0.0, 0.0
SCALE_M1_Y = 327.0
SCALE_M2_Z = -450.0

_motion_in_progress = False
_last_commanded = {1: None, 2: None, 3: None, 4: None}
DEADBAND_DEG = 1.5

async def move_robot_arm_to_pose(x, y, z, mc,
                                  velocity_limit=0.06,
                                  accel_limit=0.08,
                                  torque_limit=8):
    global _motion_in_progress
    if _motion_in_progress:
        return
    _motion_in_progress = True
    try:
        m1 = max(-180.0, min(180.0, REF_M1 + SCALE_M1_Y * (y - 0.0)))
        m2 = max(-90.0,  min(30.0,  REF_M2 + SCALE_M2_Z * (z - 0.20)))
        m3 = 0.0
        m4 = 0.0
        for motor_id, angle in [(1, m1), (2, m2), (3, m3), (4, m4)]:
            last = _last_commanded[motor_id]
            if last is not None and abs(angle - last) < DEADBAND_DEG:
                continue
            _last_commanded[motor_id] = angle
            try:
                await asyncio.wait_for(
                    mc.set_motor_angle(motor_id, angle,
                                       velocity_limit, accel_limit, torque_limit),
                    timeout=MOTOR_TIMEOUTS.get(motor_id, 1.0))
            except asyncio.TimeoutError:
                print(f"[ERROR] Motor {motor_id} timed out")
                return
    except ValueError as err:
        print(f"[DIRECT ERROR] {err}")
    finally:
        _motion_in_progress = False

class ArmControlNode(Node):
    def __init__(self):
        super().__init__("arm_control_node")
        self.paused    = False
        self._shutdown = False
        self.motor_controller = MoteusMotorController(MOTOR_IDS, ANGLE_LIMITS)
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()
        self.target_sub = self.create_subscription(
            RobotArmTarget, "/robot_arm_target", self._target_callback, 10)
        self.command_sub = self.create_subscription(
            String, "/arm_control_command", self._command_callback, 10)
        asyncio.run_coroutine_threadsafe(self._startup(), self._loop)
        self.get_logger().info("Arm control node started.")

    async def _startup(self):
        self.get_logger().info("[STARTUP] Calibrating zero offsets...")
        await self.motor_controller.calibrate_zero_offsets()
        await self._do_startup_lift()
        self.get_logger().info("[STARTUP] Arm ready.")

    async def _do_startup_lift(self):
        LIFT_VELOCITY = 0.06
        LIFT_ACCEL    = 0.04
        LIFT_TORQUE   = 8.0

        # Step 1 — lift motor 2 incrementally from park to origin
        self.get_logger().info("[LIFT] Lifting motor 2...")
        try:
            current_raw = await asyncio.wait_for(
                self.motor_controller.get_raw_motor_positions(2), timeout=2.0)
        except asyncio.TimeoutError:
            current_raw = PARK_POSE_RAW[2]
            self.get_logger().warn("[LIFT] Motor 2 read timed out, using park raw.")

        target_raw = ORIGIN_POSE_RAW[2]
        delta = target_raw - current_raw
        step_size = 0.02
        step = step_size * (1 if delta > 0 else -1)
        steps = int(abs(delta) / step_size) + 1
        self.get_logger().info(f"[LIFT] Motor 2: {current_raw:.4f} -> {target_raw:.4f} ({steps} steps)")

        for i in range(steps):
            intermediate = current_raw + (i + 1) * step
            if (step > 0 and intermediate > target_raw) or                (step < 0 and intermediate < target_raw):
                intermediate = target_raw
            try:
                await asyncio.wait_for(
                    self.motor_controller.set_motor_position(
                        2, intermediate,
                        velocity_limit=LIFT_VELOCITY,
                        accel_limit=LIFT_ACCEL,
                        torque_limit=LIFT_TORQUE),
                    timeout=3.0)
                await asyncio.sleep(0.3)
            except asyncio.TimeoutError:
                self.get_logger().error(f"[LIFT] Motor 2 step {i} timed out.")
                break
        self.get_logger().info("[LIFT] Motor 2 at origin raw position.")

        # Step 2 — move motors 1, 3, 4 to origin raw positions
        for motor_id in [1, 3, 4]:
            self.get_logger().info(f"[LIFT] Moving motor {motor_id} to origin...")
            try:
                await asyncio.wait_for(
                    self.motor_controller.set_motor_position(
                        motor_id, ORIGIN_POSE_RAW[motor_id],
                        velocity_limit=LIFT_VELOCITY,
                        accel_limit=LIFT_ACCEL,
                        torque_limit=LIFT_TORQUE),
                    timeout=8.0)
                await asyncio.sleep(0.5)
            except asyncio.TimeoutError:
                self.get_logger().error(f"[LIFT] Motor {motor_id} timed out.")
        self.get_logger().info("[LIFT] Origin pose reached.")

    def _target_callback(self, msg):
        if self.paused or self._shutdown:
            return
        asyncio.run_coroutine_threadsafe(
            move_robot_arm_to_pose(msg.x, msg.y, msg.z, self.motor_controller),
            self._loop)

    def _command_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "park":
            asyncio.run_coroutine_threadsafe(
                self._do_park(stop_after=False), self._loop)
        elif cmd == "quit":
            asyncio.run_coroutine_threadsafe(self._do_quit(), self._loop)
        elif cmd == "pause":
            self.paused = True
            self.get_logger().info("[CMD] Paused.")
        elif cmd == "resume":
            if not self._shutdown:
                self.paused = False
                self.get_logger().info("[CMD] Resumed.")

    async def _do_park(self, stop_after=False):
        self.get_logger().info("[PARK] Parking...")
        # Hold motor 4 at current position while others park
        try:
            raw4 = await asyncio.wait_for(
                self.motor_controller.get_raw_motor_positions(4), timeout=2.0)
            self.get_logger().info(f"[PARK] Holding motor 4 at raw={raw4:.4f}")
            await self.motor_controller.set_motor_position(
                4, raw4,
                velocity_limit=PARK_VELOCITY_LIMIT,
                accel_limit=PARK_ACCEL_LIMIT,
                torque_limit=PARK_TORQUE_LIMIT)
        except Exception as e:
            self.get_logger().warn(f"[PARK] Could not hold motor 4: {e}")
        # Park motors 2, 1, 3 first
        for motor_id in [2, 1, 3]:
            try:
                await asyncio.wait_for(
                    self.motor_controller.set_motor_position(
                        motor_id, PARK_POSE_RAW[motor_id],
                        velocity_limit=PARK_VELOCITY_LIMIT,
                        accel_limit=PARK_ACCEL_LIMIT,
                        torque_limit=PARK_TORQUE_LIMIT),
                    timeout=PARK_TIMEOUT_PER_MOTOR)
                await asyncio.sleep(0.5)
            except asyncio.TimeoutError:
                self.get_logger().error(f"[PARK] Motor {motor_id} timed out.")
        # Now move motor 4 to park
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    4, PARK_POSE_RAW[4],
                    velocity_limit=PARK_VELOCITY_LIMIT,
                    accel_limit=PARK_ACCEL_LIMIT,
                    torque_limit=PARK_TORQUE_LIMIT),
                timeout=PARK_TIMEOUT_PER_MOTOR)
            await asyncio.sleep(0.5)
        except asyncio.TimeoutError:
            self.get_logger().error("[PARK] Motor 4 timed out.")
        self.get_logger().info("[PARK] Parked.")
        if stop_after:
            await asyncio.sleep(0.5)
            await self.motor_controller.stop_all()
            self.get_logger().info("[PARK] Motors stopped.")

    async def _do_quit(self):
        await self._do_park(stop_after=True)

    def destroy_node(self):
        if not self._shutdown:
            self._shutdown = True
            self.paused = True
            self.get_logger().info("[SHUTDOWN] Parking arm before exit...")
            future = asyncio.run_coroutine_threadsafe(
                self._do_park(stop_after=True), self._loop)
            try:
                future.result(timeout=30.0)
                self.get_logger().info("[SHUTDOWN] Park complete, motors stopped.")
            except Exception as e:
                self.get_logger().error(f"[SHUTDOWN] Park failed: {e}")
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
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
