#!/usr/bin/env python3
import asyncio
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hand_interfaces.msg import RobotArmTarget
from hand_control.MotorController import MoteusMotorController

MOTOR_IDS = [1, 2, 3, 4, 5]

RAW_LIMITS = {
    1: (-0.2171,  0.2091),
    2: ( 0.0179,  0.2159),
    3: (-0.5031, -0.5031),  # not used during teleop
    4: ( 0.0074,  0.2836),
    5: (-0.0199,  0.0550),
}

ANGLE_LIMITS = {
    1: (-80,  100),
    2: (-85,   -2),
    3: (-80,   80),
    4: (-45,   45),
    5: (-85,   90),
}

MOTOR_TIMEOUTS = {1: 3.0, 2: 2.0, 3: 2.0, 4: 2.0, 5: 2.0}

# Origin pose in raw units
ORIGIN_POSE_RAW = {
    1:  0.0334,
    2:  0.1862,
    3: -0.5031,   # palm down — locked, never moves during teleop
    4:  0.1031,
    5: -0.0199,
}

PARK_POSE_RAW = {}  # set dynamically at startup

PARK_VELOCITY_LIMIT    = 0.15
PARK_ACCEL_LIMIT       = 0.10
PARK_TORQUE_LIMIT      = 6.0
PARK_TIMEOUT_PER_MOTOR = 4.0

M4_TUCK_RAW = 0.16     # M4 tuck — must exceed 0.1463 (measured table clearance), margin added

TRACKING_START_DELAY = 3   # seconds to wait at origin before accepting tracking targets

# ── Height (Z) → M2 mapping ───────────────────────────────────────────────────
# arm_z (hand up/down) controls M2 (shoulder lift).
# Matches safe_target_filter z_min/z_max.
HEIGHT_Z_MIN = 0.10   # arm_z — hand at lowest position
HEIGHT_Z_MAX = 0.45   # arm_z — tracker publishes up to 0.50, filter allows 0.55
HEIGHT_M2_LOW_RAW  =  0.0179  # M2 raw when arm is low  (+0.5 offset from calibration)
HEIGHT_M2_HIGH_RAW =  0.2159  # M2 raw when arm is raised

# ── Depth (X) → M4 mapping ────────────────────────────────────────────────────
# arm_x (hand front/back depth) controls M4 (elbow reach).
# Matches safe_target_filter x_min/x_max.
REACH_X_MIN = 0.20   # arm_x — hand closest to camera
REACH_X_MAX = 0.50   # arm_x — hand farthest (~80cm depth, tracker realistic max)
REACH_M4_NEAR_RAW = 0.0227  # M4 raw when hand is close (arm stretched forward)
REACH_M4_FAR_RAW  = 0.2836  # M4 raw when hand is far   (elbow folded)

# ── Height (Z) → M4 coordination ──────────────────────────────────────────────
# M4 moves with arm height (M2+M4 form a straight line at max height).
# Low raw = elbow extended; high raw = elbow retracted.
HEIGHT_M4_Z_LOW_RAW  = 0.0227  # M4 raw when arm is at Z_MIN (arm low)
HEIGHT_M4_Z_HIGH_RAW = 0.0074  # M4 raw when arm is at Z_MAX (elbow straight — max height)

# ── M4 dynamic max based on M2 height ─────────────────────────────────────────
# When arm is low, M4 is restricted to avoid hitting the table.
# Worst-case M5 (wrist fully down) is assumed — keyboard wrist control is safe.
M4_MAX_AT_M2_LOW  = 0.2516  # M4 safe max when M2 is at its lowest
M4_MAX_AT_M2_HIGH = 0.4088  # M4 safe max when M2 is at its highest (physical limit)

# ── Left/right (Y) mapping ─────────────────────────────────────────────────────
# M1 maps hand lateral position
# Measured: M1 raw range -0.2239 to 0.2810
# Camera X range (normalized 0-1, 0=left 1=right)
CAM_X_MIN = 0.2   # left edge of useful tracking zone
CAM_X_MAX = 0.8   # right edge

# ── Wrist compensation from height (Z) ────────────────────────────────────────
# Arm low (near table) → wrist pitched down; arm high → wrist level
HEIGHT_M5_LOW_RAW  =  0.0130  # raw — wrist when arm is low
HEIGHT_M5_HIGH_RAW = -0.0199  # raw — wrist when arm is raised

# Wrist manual offset — controlled by keyboard up/down
_wrist_offset_raw = 0.0
WRIST_STEP = 0.02   # raw units per keypress


def clamp_raw(motor_id, raw_pos):
    lo, hi = RAW_LIMITS[motor_id]
    if raw_pos < lo:
        return lo
    if raw_pos > hi:
        return hi
    return raw_pos


def arm_z_to_m2_raw(arm_z):
    """Map hand height (arm_z) to M2 raw position (shoulder lift)."""
    t  = (arm_z - HEIGHT_Z_MIN) / (HEIGHT_Z_MAX - HEIGHT_Z_MIN)
    t  = max(0.0, min(1.0, t))
    m2 = HEIGHT_M2_LOW_RAW + t * (HEIGHT_M2_HIGH_RAW - HEIGHT_M2_LOW_RAW)
    return clamp_raw(2, m2)


def arm_x_to_m4_raw(arm_x):
    """Map hand depth (arm_x) to M4 raw position (elbow reach)."""
    t  = (arm_x - REACH_X_MIN) / (REACH_X_MAX - REACH_X_MIN)
    t  = max(0.0, min(1.0, t))
    m4 = REACH_M4_NEAR_RAW + t * (REACH_M4_FAR_RAW - REACH_M4_NEAR_RAW)
    return clamp_raw(4, m4)


def arm_z_to_m4_raw(arm_z):
    """Map arm height to M4 raw — max height is when M2+M4 form a straight line."""
    t = (arm_z - HEIGHT_Z_MIN) / (HEIGHT_Z_MAX - HEIGHT_Z_MIN)
    t = max(0.0, min(1.0, t))
    return HEIGHT_M4_Z_LOW_RAW + t * (HEIGHT_M4_Z_HIGH_RAW - HEIGHT_M4_Z_LOW_RAW)


def m4_max_dynamic(m2_raw):
    """M4 safe maximum based on M2 height — prevents hitting table."""
    t = (m2_raw - RAW_LIMITS[2][1]) / (RAW_LIMITS[2][0] - RAW_LIMITS[2][1])
    t = max(0.0, min(1.0, t))
    return M4_MAX_AT_M2_LOW + t * (M4_MAX_AT_M2_HIGH - M4_MAX_AT_M2_LOW)


def cam_x_to_m1_raw(cam_x, zero_offset_m1):
    """Map camera X (0-1) to M1 raw position."""
    t   = (cam_x - CAM_X_MIN) / (CAM_X_MAX - CAM_X_MIN)
    t   = max(0.0, min(1.0, t))
    # Map t=0 → left limit, t=1 → right limit
    raw = RAW_LIMITS[1][1] + t * (RAW_LIMITS[1][0] - RAW_LIMITS[1][1])
    return clamp_raw(1, raw)


def arm_z_to_m5_raw(arm_z):
    """Auto wrist compensation from hand height (arm_z)."""
    t   = (arm_z - HEIGHT_Z_MIN) / (HEIGHT_Z_MAX - HEIGHT_Z_MIN)
    t   = max(0.0, min(1.0, t))
    raw = HEIGHT_M5_LOW_RAW + t * (HEIGHT_M5_HIGH_RAW - HEIGHT_M5_LOW_RAW)
    return clamp_raw(5, raw + _wrist_offset_raw)


_motion_in_progress = False
_last_commanded     = {i: None for i in MOTOR_IDS}
_last_arm_z         = None   # last Z received — used for immediate wrist updates
DEADBAND_RAW        = 0.004  # ~1.5 deg in raw units


async def move_robot_arm_to_pose(x, y, z, mc):
    """
    x = hand depth from camera (arm_x, meters) → M4 (elbow reach)
    y = hand lateral position (arm_y, ±0.25)   → M1 (left/right)
    z = hand height (arm_z, meters)             → M2 (shoulder lift) + M5 (wrist)
    """
    global _motion_in_progress, _last_arm_z
    if _motion_in_progress:
        return
    _motion_in_progress = True
    _last_arm_z = z
    try:
        # M1 — left/right (arm_y is ±0.25; y + 0.5 centres to 0.25–0.75)
        m1_raw = cam_x_to_m1_raw(y + 0.5, mc.zero_offsets.get(1, 0.0))

        # M2 — shoulder lift from hand height (Z)
        m2_raw = arm_z_to_m2_raw(z)

        # M4 — driven by Z (height); capped by dynamic safety limit
        m4_z_contrib = arm_z_to_m4_raw(z)
        m4_dyn_max   = m4_max_dynamic(m2_raw)
        m4_raw       = clamp_raw(4, min(m4_z_contrib, m4_dyn_max))

        # M3 — never commanded during teleop; held at startup position by last park command
        # M5 — wrist pitch compensation from hand height (Z) + manual offset
        m5_raw = arm_z_to_m5_raw(z)

        # Per-motor velocity/accel limits for teleop
        # M4 needs a higher velocity to keep up with Z-driven position changes
        motor_params = {
            1: (0.08, 0.08),
            2: (0.08, 0.08),
            4: (0.10, 0.08),
            5: (0.06, 0.06),
        }

        for motor_id, raw in [(1, m1_raw), (2, m2_raw),
                               (4, m4_raw), (5, m5_raw)]:
            last = _last_commanded[motor_id]
            if last is not None and abs(raw - last) < DEADBAND_RAW:
                continue
            _last_commanded[motor_id] = raw
            vel, acc = motor_params[motor_id]
            try:
                await asyncio.wait_for(
                    mc.set_motor_position(motor_id, raw,
                                          velocity_limit=vel,
                                          accel_limit=acc,
                                          torque_limit=15.0),
                    timeout=MOTOR_TIMEOUTS.get(motor_id, 1.0))
            except asyncio.TimeoutError:
                print(f"[ERROR] Motor {motor_id} timed out")
                return
    except Exception as err:
        print(f"[ERROR] move_robot_arm_to_pose: {err}")
    finally:
        _motion_in_progress = False


class ArmControlNode(Node):
    def __init__(self):
        super().__init__("arm_control_node")
        self.paused           = True   # held until startup lift completes
        self._startup_complete = False
        self._shutdown         = False
        self.motor_controller = MoteusMotorController(MOTOR_IDS, ANGLE_LIMITS)
        self._loop   = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()
        self.target_sub = self.create_subscription(
            RobotArmTarget, "/robot_arm_target", self._target_callback, 10)
        self.command_sub = self.create_subscription(
            String, "/arm_control_command", self._command_callback, 10)
        asyncio.run_coroutine_threadsafe(self._startup(), self._loop)
        self.get_logger().info("Arm control node started.")

    async def _startup(self):
        self.get_logger().info("[STARTUP] Clearing any motor faults...")
        await self.motor_controller.stop_all()
        await asyncio.sleep(0.3)
        self.get_logger().info("[STARTUP] Reading park pose from current arm position...")
        await self._read_park_pose()
        self.get_logger().info(f"[STARTUP] Park pose saved: { {k: f'{v:.4f}' for k,v in PARK_POSE_RAW.items()} }")
        success = await self._do_startup_lift()
        if not success:
            self.get_logger().error("[STARTUP] Lift failed — tracking NOT enabled. Fix arm position and restart.")
            return
        self._startup_complete = True
        for i in range(TRACKING_START_DELAY, 0, -1):
            self.get_logger().info(f"[STARTUP] Tracking starts in {i}s...")
            await asyncio.sleep(1.0)
        self.paused = False
        self.get_logger().info("[STARTUP] Arm ready — tracking enabled.")

    async def _wait_until_settled(self, targets: dict, threshold=0.012, timeout=3.0):
        """Poll motor positions until all are within threshold of their targets."""
        deadline = self._loop.time() + timeout
        while self._loop.time() < deadline:
            all_settled = True
            for motor_id, target in targets.items():
                try:
                    current = await asyncio.wait_for(
                        self.motor_controller.get_raw_motor_positions(motor_id),
                        timeout=0.5)
                    if abs(current - target) > threshold:
                        all_settled = False
                        break
                except asyncio.TimeoutError:
                    all_settled = False
                    break
            if all_settled:
                return
            await asyncio.sleep(0.1)
        self.get_logger().warn("[SETTLE] Timed out waiting for motors to settle.")

    async def _read_park_pose(self):
        for motor_id in MOTOR_IDS:
            try:
                raw = await asyncio.wait_for(
                    self.motor_controller.get_raw_motor_positions(motor_id),
                    timeout=2.0)
                PARK_POSE_RAW[motor_id] = raw
                self.get_logger().info(f"[PARK] M{motor_id} park raw={raw:.4f}")
            except asyncio.TimeoutError:
                PARK_POSE_RAW[motor_id] = 0.0
                self.get_logger().warn(f"[PARK] M{motor_id} read timed out.")

    async def _do_startup_lift(self):
        LIFT_VELOCITY = 0.10
        LIFT_ACCEL    = 0.12
        LIFT_TORQUE   = 20.0
        LIFT_SETTLE   = 1.0

        # Step 1 — lock M1/M5 stiff at park pose; M4 left floating (table-constrained flat)
        self.get_logger().info("[LIFT] Locking M1/M5 at park pose (M4 stays free)...")
        for motor_id in [1, 5]:
            raw = PARK_POSE_RAW.get(motor_id, 0.0)
            try:
                await asyncio.wait_for(
                    self.motor_controller.set_motor_position(
                        motor_id, raw, 0.05, 0.03, LIFT_TORQUE),
                    timeout=3.0)
            except asyncio.TimeoutError:
                self.get_logger().warn(f"[LIFT] M{motor_id} lock timed out.")
        # M3 — hold at exact startup position, never moves
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    3, PARK_POSE_RAW.get(3, 0.0), 0.05, 0.03, LIFT_TORQUE),
                timeout=3.0)
        except asyncio.TimeoutError:
            self.get_logger().warn("[LIFT] M3 hold timed out.")

        # Step 2 — lift M2 incrementally with everything else stiff
        # Park pose is outside operating RAW_LIMITS so no clamp on intermediate steps
        self.get_logger().info("[LIFT] Lifting M2 with arm stiff...")
        current_raw = PARK_POSE_RAW.get(2, 0.0)
        target_raw  = ORIGIN_POSE_RAW[2]
        step        = 0.02
        direction   = -1 if target_raw < current_raw else 1
        steps       = int(abs(target_raw - current_raw) / step) + 1

        for i in range(steps):
            intermediate = current_raw + direction * (i + 1) * step
            intermediate = max(target_raw, intermediate) if direction == -1 \
                           else min(target_raw, intermediate)
            try:
                await asyncio.wait_for(
                    self.motor_controller.set_motor_position(
                        2, intermediate, LIFT_VELOCITY, LIFT_ACCEL, LIFT_TORQUE),
                    timeout=3.0)
                await asyncio.sleep(0.3)
            except asyncio.TimeoutError:
                self.get_logger().error(f"[LIFT] M2 step {i} timed out.")
                break

        actual_m2 = await self.motor_controller.get_raw_motor_positions(2)
        expected_m2 = ORIGIN_POSE_RAW[2]
        if abs(actual_m2 - expected_m2) > 0.05:
            self.get_logger().error(
                f"[LIFT] M2 DID NOT MOVE — actual={actual_m2:.4f} expected={expected_m2:.4f}. "
                f"Arm may be mechanically blocked. Aborting startup.")
            return False
        self.get_logger().info(f"[LIFT] M2 at {actual_m2:.4f} — settling {LIFT_SETTLE}s...")
        await asyncio.sleep(LIFT_SETTLE)

        # Step 3 — tuck M4 now that arm is elevated and table clearance is safe
        self.get_logger().info("[LIFT] Tucking M4...")
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    4, M4_TUCK_RAW, 0.10, 0.06, LIFT_TORQUE),
                timeout=4.0)
            await asyncio.sleep(0.8)
        except asyncio.TimeoutError:
            self.get_logger().warn("[LIFT] M4 tuck timed out.")

        # Step 4 — move M1/M4/M5 to origin (M3 intentionally skipped — never rotated)
        self.get_logger().info("[LIFT] Moving to origin pose...")
        for motor_id in [1, 4, 5]:
            raw = clamp_raw(motor_id, ORIGIN_POSE_RAW[motor_id])
            try:
                await asyncio.wait_for(
                    self.motor_controller.set_motor_position(
                        motor_id, raw, 0.12, 0.08, LIFT_TORQUE),
                    timeout=5.0)
                await asyncio.sleep(0.3)
            except asyncio.TimeoutError:
                self.get_logger().error(f"[LIFT] M{motor_id} timed out.")

        # Step 5 — M2 final origin
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    2, clamp_raw(2, ORIGIN_POSE_RAW[2]), 0.12, 0.08, LIFT_TORQUE),
                timeout=5.0)
        except asyncio.TimeoutError:
            self.get_logger().error("[LIFT] M2 final origin timed out.")

        self.get_logger().info("[LIFT] Origin pose reached.")
        return True

    def _target_callback(self, msg):
        if self.paused or self._shutdown:
            return
        m2 = arm_z_to_m2_raw(msg.z)
        self.get_logger().info(f"[TELEOP] x={msg.x:.3f} y={msg.y:.3f} z={msg.z:.3f} → M2={m2:.4f}")
        asyncio.run_coroutine_threadsafe(
            move_robot_arm_to_pose(msg.x, msg.y, msg.z, self.motor_controller),
            self._loop)

    def _command_callback(self, msg):
        global _wrist_offset_raw
        cmd = msg.data.strip().lower()
        if cmd == "park":
            asyncio.run_coroutine_threadsafe(
                self._do_park(stop_after=True), self._loop)
        elif cmd == "quit":
            asyncio.run_coroutine_threadsafe(self._do_quit(), self._loop)
        elif cmd == "stop":
            self._shutdown = True
            self.paused = True
            asyncio.run_coroutine_threadsafe(
                self.motor_controller.stop_all(), self._loop)
            self.get_logger().info("[CMD] Motors stopped immediately.")
        elif cmd == "pause":
            self.paused = True
            self.get_logger().info("[CMD] Paused.")
        elif cmd == "resume":
            if not self._shutdown and self._startup_complete:
                self.paused = False
                self.get_logger().info("[CMD] Resumed.")
        elif cmd == "wrist_up":
            _wrist_offset_raw = min(0.15, _wrist_offset_raw + WRIST_STEP)
            self.get_logger().info(f"[WRIST] up  offset={_wrist_offset_raw:.3f}")
            asyncio.run_coroutine_threadsafe(self._apply_wrist(), self._loop)
        elif cmd == "wrist_down":
            _wrist_offset_raw = max(-0.25, _wrist_offset_raw - WRIST_STEP)
            self.get_logger().info(f"[WRIST] dn  offset={_wrist_offset_raw:.3f}")
            asyncio.run_coroutine_threadsafe(self._apply_wrist(), self._loop)

    async def _apply_wrist(self):
        """Command M5 immediately after a wrist_up/wrist_down keypress."""
        z = _last_arm_z if _last_arm_z is not None else (HEIGHT_Z_MIN + HEIGHT_Z_MAX) / 2.0
        m5_raw = arm_z_to_m5_raw(z)
        _last_commanded[5] = m5_raw
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    5, m5_raw, velocity_limit=0.10, accel_limit=0.08, torque_limit=8.0),
                timeout=2.0)
        except asyncio.TimeoutError:
            self.get_logger().warn('[WRIST] M5 command timed out')

    async def _do_park(self, stop_after=False):
        self.get_logger().info("[PARK] Parking...")
        self.paused = True

        # Step 1 — always tuck M4 before lowering M2 — unsafe at any arm height if extended
        self.get_logger().info("[PARK] Tucking M4...")
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    4, M4_TUCK_RAW,
                    PARK_VELOCITY_LIMIT, PARK_ACCEL_LIMIT, PARK_TORQUE_LIMIT),
                timeout=4.0)
        except asyncio.TimeoutError:
            self.get_logger().warn("[PARK] M4 tuck timed out.")
        await self._wait_until_settled({4: M4_TUCK_RAW})

        # Step 2 — move M1/M5 to park pose; return M3 to exact startup position (no clamp)
        m1_target = clamp_raw(1, PARK_POSE_RAW.get(1, 0.0))
        m5_target = clamp_raw(5, PARK_POSE_RAW.get(5, 0.0))
        m3_target = PARK_POSE_RAW.get(3, 0.0)
        for motor_id, raw in [(1, m1_target), (5, m5_target)]:
            try:
                await asyncio.wait_for(
                    self.motor_controller.set_motor_position(
                        motor_id, raw,
                        PARK_VELOCITY_LIMIT, PARK_ACCEL_LIMIT, PARK_TORQUE_LIMIT),
                    timeout=PARK_TIMEOUT_PER_MOTOR)
            except asyncio.TimeoutError:
                self.get_logger().error(f"[PARK] M{motor_id} timed out.")
        # M3 — return to exact startup position, bypassing clamp_raw
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    3, m3_target,
                    PARK_VELOCITY_LIMIT, PARK_ACCEL_LIMIT, PARK_TORQUE_LIMIT),
                timeout=PARK_TIMEOUT_PER_MOTOR)
        except asyncio.TimeoutError:
            self.get_logger().error("[PARK] M3 timed out.")

        # Wait until M1/M3/M5 have physically reached their park positions
        self.get_logger().info("[PARK] Waiting for M1/M3/M5 to settle...")
        await self._wait_until_settled({1: m1_target, 3: m3_target, 5: m5_target})

        # Step 3 — lower M2 slowly
        self.get_logger().info("[PARK] Lowering M2...")
        try:
            current = await asyncio.wait_for(
                self.motor_controller.get_raw_motor_positions(2), timeout=2.0)
        except asyncio.TimeoutError:
            current = ORIGIN_POSE_RAW[2]

        target    = PARK_POSE_RAW.get(2, 0.0)
        step      = 0.02
        direction = 1 if target > current else -1
        steps     = int(abs(target - current) / step) + 1

        for i in range(steps):
            intermediate = current + direction * (i + 1) * step
            intermediate = min(intermediate, target) if direction == 1 \
                           else max(intermediate, target)
            try:
                await asyncio.wait_for(
                    self.motor_controller.set_motor_position(
                        2, intermediate,
                        PARK_VELOCITY_LIMIT, PARK_ACCEL_LIMIT, PARK_TORQUE_LIMIT),
                    timeout=3.0)
                await asyncio.sleep(0.4)
            except asyncio.TimeoutError:
                self.get_logger().error(f"[PARK] M2 step {i} timed out.")
                break

        # Step 4 — M4 to final park position (M2 is now at table level)
        raw4 = clamp_raw(4, PARK_POSE_RAW.get(4, 0.0))
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    4, raw4,
                    PARK_VELOCITY_LIMIT, PARK_ACCEL_LIMIT, PARK_TORQUE_LIMIT),
                timeout=4.0)
        except asyncio.TimeoutError:
            self.get_logger().warn("[PARK] M4 final timed out.")
        await self._wait_until_settled({4: raw4})

        self.get_logger().info("[PARK] Parked.")
        if stop_after:
            await self.motor_controller.stop_all()
            self._shutdown = True
            self.get_logger().info("[PARK] Motors stopped.")

    async def _do_quit(self):
        self._shutdown = True
        await self._do_park(stop_after=True)
        rclpy.shutdown()

    def destroy_node(self):
        if not self._shutdown:
            self._shutdown = True
            self.paused    = True
            self.get_logger().info("[SHUTDOWN] Ctrl+C — parking arm slowly...")
            future = asyncio.run_coroutine_threadsafe(
                self._do_park(stop_after=False), self._loop)
            try:
                future.result(timeout=40.0)
                self.get_logger().info("[SHUTDOWN] Park complete.")
            except Exception as e:
                self.get_logger().error(f"[SHUTDOWN] Park failed: {e}")
        # Always stop motors as final step — guaranteed regardless of how we got here
        self.get_logger().info("[SHUTDOWN] Stopping motors...")
        stop_future = asyncio.run_coroutine_threadsafe(
            self.motor_controller.stop_all(), self._loop)
        try:
            stop_future.result(timeout=5.0)
            self.get_logger().info("[SHUTDOWN] Motors stopped.")
        except Exception as e:
            self.get_logger().error(f"[SHUTDOWN] Motor stop failed: {e}")
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
