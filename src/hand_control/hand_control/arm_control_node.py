#!/usr/bin/env python3
import asyncio
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hand_interfaces.msg import RobotArmTarget
from hand_control.MotorController import MoteusMotorController

MOTOR_IDS = [1, 2, 3, 4]
ANGLE_LIMITS = {1: (-180, 180), 2: (-90, 30), 3: (-80, 80), 4: (-90, 90)}
MOTOR_TIMEOUTS = {1: 3.0, 2: 1.0, 3: 1.0, 4: 1.0}

PARK_VELOCITY_LIMIT    = 0.25
PARK_ACCEL_LIMIT       = 0.15
PARK_TORQUE_LIMIT      = 6.0
PARK_TIMEOUT_PER_MOTOR = 3.0  # increased to give park time to complete

PARK_POSE_DEG         = {1: 0.0,   2: 0.0,    3: 0.0,   4: 0.0}
INTERMEDIATE_POSE_DEG = {1: 6.70,  2: -75.28, 3: -2.16, 4: -106.81}
ORIGIN_POSE_DEG       = {1: 90.00, 2: -49.18, 3: 4.03,  4: 3.06}

REF_X, REF_Y, REF_Z             = 0.20, 0.0, 0.24
REF_M1, REF_M2, REF_M3, REF_M4 = 99.0, -53.0, 0.0, -17.0
SCALE_M1_Y = 200.0
SCALE_M2_Z = -200.0
SCALE_M4_X = -200.0

_motion_in_progress = False

async def move_robot_arm_to_pose(x, y, z, mc,
                                  velocity_limit=0.06,
                                  accel_limit=0.08,
                                  torque_limit=8):
    global _motion_in_progress
    if _motion_in_progress:
        return
    _motion_in_progress = True
    try:
        m1 = max(-180.0, min(180.0, REF_M1 + SCALE_M1_Y * (y - REF_Y)))
        m2 = max(-90.0,  min(30.0,  REF_M2 + SCALE_M2_Z * (z - REF_Z)))
        m3 = max(-60.0,  min(60.0,  REF_M3))
        m4 = max(-90.0,  min(90.0,  REF_M4 + SCALE_M4_X * (x - REF_X)))
        for motor_id, angle in [(1, m1), (2, m2), (3, m3), (4, m4)]:
            try:
                await asyncio.wait_for(
                    mc.set_motor_angle(motor_id, angle,
                                       velocity_limit, accel_limit, torque_limit),
                    timeout=MOTOR_TIMEOUTS.get(motor_id, 1.0))
            except asyncio.TimeoutError:
                print(f'[ERROR] Motor {motor_id} timed out')
                return
    except ValueError as err:
        print(f'[DIRECT ERROR] {err}')
    finally:
        _motion_in_progress = False

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        self.paused    = False
        self._shutdown = False
        self.motor_controller = MoteusMotorController(MOTOR_IDS, ANGLE_LIMITS)
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()
        self.target_sub = self.create_subscription(
            RobotArmTarget, '/robot_arm_target', self._target_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/arm_control_command', self._command_callback, 10)
        asyncio.run_coroutine_threadsafe(self._startup(), self._loop)
        self.get_logger().info('Arm control node started.')

    async def _startup(self):
        self.get_logger().info('[STARTUP] Calibrating zero offsets...')
        await self.motor_controller.calibrate_zero_offsets()
        await self._do_startup_lift()
        self.get_logger().info('[STARTUP] Arm ready.')

    async def _do_startup_lift(self):
        LIFT_DELTA, LIFT_VELOCITY, LIFT_ACCEL, LIFT_TORQUE = -0.18, 0.06, 0.04, 8.0
        self.get_logger().info('[LIFT] Lifting motor 2...')
        try:
            current_raw = await asyncio.wait_for(
                self.motor_controller.get_raw_motor_positions(2), timeout=2.0)
        except asyncio.TimeoutError:
            current_raw = self.motor_controller.zero_offsets.get(2, 0.0)
            self.get_logger().warn('[LIFT] Motor 2 read timed out, using offset.')
        target_raw = current_raw + LIFT_DELTA
        step = 0.02
        steps = int(abs(LIFT_DELTA) / step) + 1
        for i in range(steps):
            intermediate = max(current_raw + (i + 1) * (-step), target_raw)
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
                self.get_logger().error(f'[LIFT] Motor 2 step {i} timed out.')
                break
        motor4_zero = self.motor_controller.zero_offsets.get(4, 0.0)
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    4, motor4_zero,
                    velocity_limit=0.10, accel_limit=0.08, torque_limit=LIFT_TORQUE),
                timeout=5.0)
        except asyncio.TimeoutError:
            self.get_logger().error('[LIFT] Motor 4 home timed out.')
        await self._move_to_pose_deg(INTERMEDIATE_POSE_DEG, label='LIFT',
                                      velocity=0.12, accel=0.08,
                                      torque=LIFT_TORQUE, timeout=5.0)
        await self._move_to_pose_deg(ORIGIN_POSE_DEG, label='LIFT',
                                      velocity=0.12, accel=0.08,
                                      torque=LIFT_TORQUE, timeout=5.0)
        self.get_logger().info('[LIFT] Origin pose reached.')

    async def _move_to_pose_deg(self, pose_deg, label='MOVE',
                                 velocity=0.12, accel=0.08,
                                 torque=8.0, timeout=5.0):
        for motor_id, angle in pose_deg.items():
            try:
                await asyncio.wait_for(
                    self.motor_controller.set_motor_angle(
                        motor_id, angle,
                        velocity_limit=velocity,
                        accel_limit=accel,
                        torque_limit=torque),
                    timeout=timeout)
            except asyncio.TimeoutError:
                self.get_logger().error(f'[{label}] Motor {motor_id} timed out.')

    def _target_callback(self, msg):
        if self.paused or self._shutdown:
            return
        asyncio.run_coroutine_threadsafe(
            move_robot_arm_to_pose(msg.x, msg.y, msg.z, self.motor_controller),
            self._loop)

    def _command_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'park':
            asyncio.run_coroutine_threadsafe(
                self._do_park(stop_after=False), self._loop)
        elif cmd == 'quit':
            asyncio.run_coroutine_threadsafe(self._do_quit(), self._loop)
        elif cmd == 'pause':
            self.paused = True
            self.get_logger().info('[CMD] Paused.')
        elif cmd == 'resume':
            if not self._shutdown:
                self.paused = False
                self.get_logger().info('[CMD] Resumed.')

    async def _do_park(self, stop_after=False):
        """Move each motor to park pose one at a time, then optionally stop."""
        self.get_logger().info('[PARK] Parking...')
        # Move motors sequentially — 2 first since it holds the arm up
        for motor_id, angle in [(2, PARK_POSE_DEG[2]),
                                  (1, PARK_POSE_DEG[1]),
                                  (4, PARK_POSE_DEG[4]),
                                  (3, PARK_POSE_DEG[3])]:
            try:
                await asyncio.wait_for(
                    self.motor_controller.set_motor_angle(
                        motor_id, angle,
                        velocity_limit=PARK_VELOCITY_LIMIT,
                        accel_limit=PARK_ACCEL_LIMIT,
                        torque_limit=PARK_TORQUE_LIMIT),
                    timeout=PARK_TIMEOUT_PER_MOTOR)
                await asyncio.sleep(0.5)  # wait between each motor
            except asyncio.TimeoutError:
                self.get_logger().error(f'[PARK] Motor {motor_id} timed out.')
        self.get_logger().info('[PARK] Parked.')
        if stop_after:
            await asyncio.sleep(0.5)  # brief pause before cutting torque
            await self.motor_controller.stop_all()
            self.get_logger().info('[PARK] Motors stopped.')

    async def _do_quit(self):
        await self._do_park(stop_after=True)

    def destroy_node(self):
        """Park arm safely before shutting down."""
        if not self._shutdown:
            self._shutdown = True
            self.paused = True
            self.get_logger().info('[SHUTDOWN] Parking arm before exit...')
            future = asyncio.run_coroutine_threadsafe(
                self._do_park(stop_after=True), self._loop)
            try:
                future.result(timeout=30.0)
                self.get_logger().info('[SHUTDOWN] Park complete, motors stopped.')
            except Exception as e:
                self.get_logger().error(f'[SHUTDOWN] Park failed: {e}')
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
