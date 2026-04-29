#!/usr/bin/env python3
import asyncio
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hand_interfaces.msg import RobotArmTarget
from hand_control.MotorController import MoteusMotorController
try:
    from serial.serialutil import SerialException
except ImportError:
    SerialException = OSError

MOTOR_IDS = [1, 2, 3, 4, 5]

RAW_LIMITS = {
    1: (-0.2171,  0.2091),
    2: ( 0.0512,  0.2159),   # M2 min raised to 0.0512
    3: (-0.7531, -0.2531),  # ±90° from locked pose
    4: (-0.1539,  0.0443),   # M4 hard limits from measurement
    5: (-0.40,  0.15),
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

PARK_VELOCITY_LIMIT    = 0.08   # M1/M3/M4/M5 — need to reach park pose before M2 lowers
PARK_ACCEL_LIMIT       = 0.06
PARK_M2_VELOCITY       = 0.025  # M2 lowering — slow to avoid hitting hand/table
PARK_M2_ACCEL          = 0.015
PARK_TORQUE_LIMIT      = 6.0    # M1/M3/M4/M5
PARK_M2_TORQUE         = 15.0   # M2 needs full torque to resist gravity during descent
PARK_TIMEOUT_PER_MOTOR = 8.0

M4_TUCK_RAW = 0.0443   # M4 tuck — clamped to new hard limit

TRACKING_START_DELAY = 3   # seconds to wait at origin before accepting tracking targets

# ── Height (Z) → M2 mapping ───────────────────────────────────────────────────
# arm_z (hand up/down) controls M2 (shoulder lift).
# Matches safe_target_filter z_min/z_max.
HEIGHT_Z_MIN = 0.10   # arm_z — hand at lowest position
HEIGHT_Z_MAX = 0.45   # arm_z — tracker publishes up to 0.50, filter allows 0.55
HEIGHT_M2_LOW_RAW  =  0.0512  # M2 raw when arm is low  (hard min)
HEIGHT_M2_HIGH_RAW =  0.2159  # M2 raw when arm is raised

# ── Depth (X) → M4 mapping ────────────────────────────────────────────────────
# arm_x (hand front/back depth) controls M4 (elbow reach).
# Matches safe_target_filter x_min/x_max.
REACH_X_MIN = 0.20   # arm_x — hand closest to camera
REACH_X_MAX = 0.50   # arm_x — hand farthest (~80cm depth, tracker realistic max)
REACH_M4_NEAR_RAW = -0.1539  # M4 raw when hand is close (arm extended)
REACH_M4_FAR_RAW  =  0.0443  # M4 raw when hand is far   (arm tucked)

# ── Height (Z) → M4 coordination ──────────────────────────────────────────────
HEIGHT_M4_Z_LOW_RAW  = 0.0227  # M4 raw when arm is at Z_MIN (arm low)
HEIGHT_M4_Z_HIGH_RAW = 0.0074  # M4 raw when arm is at Z_MAX

# ── M4 dynamic max based on M2 height ─────────────────────────────────────────
M4_MAX_AT_M2_LOW  = 0.0443  # M4 safe max when M2 is at its lowest (hard limit)
M4_MAX_AT_M2_HIGH = 0.0443  # M4 safe max when M2 is at its highest (hard limit)

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

# Wrist manual offset — controlled by keyboard W/S
_wrist_offset_raw = 0.0
WRIST_STEP = 0.02   # raw units per keypress

# Elbow (M4) absolute position — controlled by keyboard 3/4
_elbow_raw  = None   # set to home pre-position after startup
ELBOW_STEP  = 0.02

# Palm rotation (M3) absolute position — controlled by keyboard 5/6
_m3_raw  = -0.5031   # starts at locked pose
M3_STEP  = 0.02


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


def arm_y_to_m1_raw(arm_y):
    """Map hand lateral position (arm_y, -0.30 to +0.30) to M1 raw position."""
    t   = (arm_y - (-0.30)) / (0.30 - (-0.30))
    t   = max(0.0, min(1.0, t))
    raw = RAW_LIMITS[1][1] + t * (RAW_LIMITS[1][0] - RAW_LIMITS[1][1])
    return clamp_raw(1, raw)


def arm_z_to_m5_raw(arm_z):
    """Auto wrist compensation from hand height (arm_z)."""
    t   = (arm_z - HEIGHT_Z_MIN) / (HEIGHT_Z_MAX - HEIGHT_Z_MIN)
    t   = max(0.0, min(1.0, t))
    raw = HEIGHT_M5_LOW_RAW + t * (HEIGHT_M5_HIGH_RAW - HEIGHT_M5_LOW_RAW)
    return clamp_raw(5, raw + _wrist_offset_raw)


_last_commanded     = {i: None for i in MOTOR_IDS}
_last_arm_z         = None   # last Z received — used for immediate wrist updates
DEADBAND_RAW        = 0.002  # ~0.7 deg — reduced so small M2/M1 changes aren't filtered

# Latest target written by ROS callback, consumed by tracking loop.
# Never queued — always the most recent position.
_pending_target     = None   # tuple (x, y, z) or None
_pending_lock       = threading.Lock()


async def _tracking_loop(mc):
    """Consume the latest pending target at up to 20 Hz. Never queues stale positions."""
    global _last_arm_z, _pending_target
    while True:
        await asyncio.sleep(0.05)   # 20 Hz poll
        with _pending_lock:
            target = _pending_target
            _pending_target = None
        if target is None:
            continue
        x, y, z = target
        _last_arm_z = z
        try:
            targets = [
                (1, arm_y_to_m1_raw(y),    0.04, 0.03),  # Y → M1
                (2, arm_z_to_m2_raw(z),    0.04, 0.03),  # Z → M2
            ]
            needed = [
                (mid, raw, vel, acc)
                for mid, raw, vel, acc in targets
                if _last_commanded[mid] is None or abs(raw - _last_commanded[mid]) >= DEADBAND_RAW
            ]
            if not needed:
                continue
            for mid, raw, _, _ in needed:
                _last_commanded[mid] = raw
            print(f"[TRACK] commanding {[(m,f'{r:.4f}') for m,r,_,_ in needed]}")
            results = await asyncio.gather(*[
                asyncio.wait_for(
                    mc.set_motor_position(mid, raw, vel, acc, 15.0),
                    timeout=MOTOR_TIMEOUTS.get(mid, 1.0))
                for mid, raw, vel, acc in needed
            ], return_exceptions=True)
            for (mid, _, _, _), res in zip(needed, results):
                if isinstance(res, asyncio.TimeoutError):
                    print(f"[ERROR] Motor {mid} timed out")
        except Exception as err:
            print(f"[ERROR] _tracking_loop: {err}")


class ArmControlNode(Node):
    def __init__(self):
        super().__init__("arm_control_node")
        self.paused            = True   # held until startup lift completes
        self._manual_pause     = False  # set by keyboard P — blocks tracking_found resume
        self._startup_complete = False
        self._shutdown         = False
        self.motor_controller = MoteusMotorController(MOTOR_IDS, ANGLE_LIMITS)
        self._loop   = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()
        self._quit_future = None   # set when 'quit' received — destroy_node waits for it
        self._quit_done   = threading.Event()  # set after quit-park completes → exits spin loop
        self.target_sub = self.create_subscription(
            RobotArmTarget, "/robot_arm_target", self._target_callback, 10)
        self.command_sub = self.create_subscription(
            String, "/arm_control_command", self._command_callback, 10)
        asyncio.run_coroutine_threadsafe(self._startup(), self._loop)
        asyncio.run_coroutine_threadsafe(self._keepalive(), self._loop)
        asyncio.run_coroutine_threadsafe(_tracking_loop(self.motor_controller), self._loop)
        self.get_logger().info("Arm control node started.")

    async def _keepalive(self):
        """Query motor 1 every second to keep the serial link alive when paused."""
        while not self._shutdown:
            await asyncio.sleep(1.0)
            if self.paused and self._startup_complete and not self._shutdown:
                try:
                    await asyncio.wait_for(
                        self.motor_controller.get_raw_motor_positions(1), timeout=0.5)
                except Exception:
                    pass

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

    async def _wait_until_settled(self, targets: dict, threshold=0.012, timeout=6.0):
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
            except asyncio.TimeoutError:
                self.get_logger().error(f"[LIFT] M2 step {i} timed out.")
            await asyncio.sleep(0.3)
        # Always command final target
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    2, target_raw, LIFT_VELOCITY, LIFT_ACCEL, LIFT_TORQUE),
                timeout=3.0)
            await asyncio.sleep(0.5)
        except asyncio.TimeoutError:
            self.get_logger().error("[LIFT] M2 final target timed out.")

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

        # Step 6 — pre-position to tracking home so M4/M2 don't jump on first tracking command
        self.get_logger().info("[LIFT] Pre-positioning to tracking home (HOME_X=0.35, HOME_Z=0.24)...")
        home_m4 = arm_x_to_m4_raw(0.35)   # HOME_X midpoint → M4 extended to home reach
        home_m2 = arm_z_to_m2_raw(0.24)   # HOME_Z → M2 home height
        for motor_id, raw in [(4, home_m4), (2, home_m2)]:
            try:
                await asyncio.wait_for(
                    self.motor_controller.set_motor_position(
                        motor_id, raw, 0.08, 0.06, LIFT_TORQUE),
                    timeout=5.0)
            except asyncio.TimeoutError:
                self.get_logger().warn(f"[LIFT] Pre-position M{motor_id} timed out.")
        await self._wait_until_settled({4: home_m4, 2: home_m2}, threshold=0.012, timeout=8.0)
        self.get_logger().info(f"[LIFT] Home pre-position: M4={home_m4:.4f} M2={home_m2:.4f}")
        global _elbow_raw
        _elbow_raw = home_m4   # keyboard elbow control starts from here

        self.get_logger().info("[LIFT] Origin pose reached.")
        return True

    def _target_callback(self, msg):
        if self.paused or self._shutdown:
            self.get_logger().warn(
                f"[TELEOP] DROPPED (paused={self.paused} shutdown={self._shutdown}) "
                f"x={msg.x:.3f} y={msg.y:.3f} z={msg.z:.3f}")
            return
        self.get_logger().info(
            f"[TELEOP] x={msg.x:.3f} y={msg.y:.3f} z={msg.z:.3f} "
            f"→ M1={arm_y_to_m1_raw(msg.y):.4f} M2={arm_z_to_m2_raw(msg.z):.4f}")
        with _pending_lock:
            global _pending_target
            _pending_target = (msg.x, msg.y, msg.z)

    def _command_callback(self, msg):
        global _wrist_offset_raw
        cmd = msg.data.strip().lower()
        if cmd == "park":
            asyncio.run_coroutine_threadsafe(
                self._do_park(stop_after=True), self._loop)
        elif cmd == "quit":
            self._quit_future = asyncio.run_coroutine_threadsafe(self._do_quit(), self._loop)
        elif cmd == "stop":
            self._shutdown = True
            self.paused = True
            asyncio.run_coroutine_threadsafe(
                self.motor_controller.stop_all(), self._loop)
            self.get_logger().info("[CMD] Motors stopped immediately.")
        elif cmd == "pause":
            self._manual_pause = True
            self.paused = True
            self.get_logger().info("[CMD] Paused (manual).")
        elif cmd == "resume":
            if not self._shutdown and self._startup_complete:
                self._manual_pause = False
                self.paused = False
                self.get_logger().info("[CMD] Resumed (manual).")
        elif cmd == "tracking_lost":
            self.paused = True
            self.get_logger().info("[CMD] Paused (tracking lost).")
        elif cmd == "tracking_found":
            if not self._shutdown and self._startup_complete and not self._manual_pause:
                self.paused = False
                self.get_logger().info("[CMD] Resumed (tracking found).")
        elif cmd == "go_home":
            if not self._shutdown and self._startup_complete:
                self.paused = True
                asyncio.run_coroutine_threadsafe(self._do_go_home(), self._loop)
                self.get_logger().info("[CMD] Moving to home for recalibration.")
        elif cmd == "wrist_up":
            _wrist_offset_raw = min(0.15, _wrist_offset_raw + WRIST_STEP)
            self.get_logger().info(f"[WRIST] up  offset={_wrist_offset_raw:.3f}")
            asyncio.run_coroutine_threadsafe(self._apply_wrist(), self._loop)
        elif cmd == "wrist_down":
            _wrist_offset_raw = max(-0.40, _wrist_offset_raw - WRIST_STEP)
            self.get_logger().info(f"[WRIST] dn  offset={_wrist_offset_raw:.3f}")
            asyncio.run_coroutine_threadsafe(self._apply_wrist(), self._loop)
        elif cmd in ("elbow_out", "elbow_in"):
            global _elbow_raw
            if _elbow_raw is not None:
                step = -ELBOW_STEP if cmd == "elbow_out" else ELBOW_STEP
                _elbow_raw = clamp_raw(4, _elbow_raw + step)
                self.get_logger().info(f"[ELBOW] {cmd}  raw={_elbow_raw:.4f}")
                asyncio.run_coroutine_threadsafe(self._apply_elbow(), self._loop)
        elif cmd in ("palm_cw", "palm_ccw"):
            global _m3_raw
            _m3_raw = clamp_raw(3, _m3_raw + (M3_STEP if cmd == "palm_cw" else -M3_STEP))
            self.get_logger().info(f"[PALM] {cmd}  raw={_m3_raw:.4f}")
            asyncio.run_coroutine_threadsafe(self._apply_m3(), self._loop)

    async def _apply_m3(self):
        _last_commanded[3] = _m3_raw
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    3, _m3_raw, velocity_limit=0.04, accel_limit=0.03, torque_limit=15.0),
                timeout=2.0)
        except asyncio.TimeoutError:
            self.get_logger().warn('[PALM] M3 command timed out')

    async def _apply_elbow(self):
        if _elbow_raw is None:
            return
        _last_commanded[4] = _elbow_raw
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    4, _elbow_raw, velocity_limit=0.04, accel_limit=0.03, torque_limit=15.0),
                timeout=2.0)
        except asyncio.TimeoutError:
            self.get_logger().warn('[ELBOW] M4 command timed out')

    async def _apply_wrist(self):
        """Command M5 immediately after a wrist_up/wrist_down keypress."""
        m5_raw = clamp_raw(5, _wrist_offset_raw)
        _last_commanded[5] = m5_raw
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    5, m5_raw, velocity_limit=0.10, accel_limit=0.08, torque_limit=8.0),
                timeout=2.0)
        except asyncio.TimeoutError:
            self.get_logger().warn('[WRIST] M5 command timed out')

    def _serial_alive(self):
        """Return False if the fdcanusb serial device has disconnected."""
        try:
            transport = self.motor_controller._transport
            if transport is not None and hasattr(transport, '_serial'):
                return transport._serial.is_open
        except Exception:
            pass
        return True  # assume alive if we can't tell

    async def _do_go_home(self):
        """Move arm to HOME tracking position for recalibration."""
        home_m2 = arm_z_to_m2_raw(0.24)   # HOME_Z
        home_m4 = arm_x_to_m4_raw(0.35)   # HOME_X
        home_m1 = arm_y_to_m1_raw(0.0)   # HOME_Y=0 → M1 center
        home_m5 = clamp_raw(5, 0.0)
        targets = [(1, home_m1), (2, home_m2), (4, home_m4), (5, home_m5)]
        for motor_id, raw in targets:
            try:
                await asyncio.wait_for(
                    self.motor_controller.set_motor_position(
                        motor_id, raw, 0.08, 0.06, 15.0),
                    timeout=5.0)
                await asyncio.sleep(0.2)
            except asyncio.TimeoutError:
                self.get_logger().warn(f"[HOME] M{motor_id} timed out.")
        await self._wait_until_settled({1: home_m1, 2: home_m2, 4: home_m4})
        self.get_logger().info("[HOME] Arm at home — ready for recalibration.")

    async def _do_park(self, stop_after=False):
        self.get_logger().info("[PARK] Parking...")
        self.paused = True
        if not self._serial_alive():
            self.get_logger().error("[PARK] Serial device gone — skipping park.")
            return

        try:
            await self._do_park_steps(stop_after)
        except SerialException:
            self.get_logger().error("[PARK] Serial device disconnected mid-park — aborting.")
        return

    async def _do_park_steps(self, stop_after=False):
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
        await asyncio.sleep(1.5)  # hard wait — ensure M4 fully tucked before M2 lowers

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
        await self._wait_until_settled({1: m1_target, 3: m3_target, 5: m5_target}, timeout=12.0)

        # Step 3 — lower M2 slowly
        self.get_logger().info("[PARK] Lowering M2...")
        try:
            current = await asyncio.wait_for(
                self.motor_controller.get_raw_motor_positions(2), timeout=2.0)
        except asyncio.TimeoutError:
            current = ORIGIN_POSE_RAW[2]

        # Always lower to the absolute floor so M2 is resting on the table before stop_all.
        # Use full torque — velocity/accel already limit speed; low torque causes gravity-drop.
        target = RAW_LIMITS[2][0]   # 0.0512 — physical low stop
        step   = 0.01               # fine steps for gentle lowering

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
                        PARK_M2_VELOCITY, PARK_M2_ACCEL, PARK_M2_TORQUE),
                    timeout=4.0)
            except asyncio.TimeoutError:
                self.get_logger().error(f"[PARK] M2 step {i} timed out.")
            # Wait until M2 physically reaches this step before commanding the next
            await self._wait_until_settled({2: intermediate}, threshold=0.015, timeout=4.0)
        # Final command to floor — ensures we land even if last step lagged
        try:
            await asyncio.wait_for(
                self.motor_controller.set_motor_position(
                    2, target, PARK_M2_VELOCITY, PARK_M2_ACCEL, PARK_M2_TORQUE),
                timeout=4.0)
        except asyncio.TimeoutError:
            self.get_logger().error("[PARK] M2 final target timed out.")
        # Wait until fully at rest at the floor
        await self._wait_until_settled({2: target}, threshold=0.015, timeout=12.0)
        await asyncio.sleep(1.0)

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
            await asyncio.sleep(1.5)  # let M2 reach rest before cutting power
            await self.motor_controller.stop_all()
            self._shutdown = True
            self.get_logger().info("[PARK] Motors stopped.")

    async def _do_quit(self):
        self._shutdown = True
        global _pending_target
        with _pending_lock:
            _pending_target = None   # stop tracking loop immediately
        await self._do_park(stop_after=True)
        self._quit_done.set()        # signals main() spin loop to exit

    def destroy_node(self):
        if self._quit_future is not None:
            # Q was pressed — _do_quit owns the park+stop_all sequence.
            # Wait for it to finish before tearing down so stop_all() here
            # doesn't cut power mid-park.
            self.get_logger().info("[SHUTDOWN] Waiting for quit-park to complete...")
            try:
                self._quit_future.result(timeout=45.0)
                self.get_logger().info("[SHUTDOWN] Quit-park complete.")
            except Exception as e:
                self.get_logger().error(f"[SHUTDOWN] Quit-park wait failed: {e}")
        elif not self._shutdown:
            self._shutdown = True
            self.paused    = True
            with _pending_lock:
                global _pending_target
                _pending_target = None   # stop tracking loop before park starts
            self.get_logger().info("[SHUTDOWN] SIGTERM — parking arm slowly...")
            future = asyncio.run_coroutine_threadsafe(
                self._do_park(stop_after=False), self._loop)
            try:
                future.result(timeout=40.0)
                self.get_logger().info("[SHUTDOWN] Park complete.")
            except SerialException:
                self.get_logger().error("[SHUTDOWN] Serial device disconnected — skipping park.")
            except Exception as e:
                self.get_logger().error(f"[SHUTDOWN] Park failed: {e}")
            # stop_all after SIGTERM-triggered park
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
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node._quit_done.is_set():
            executor.spin_once(timeout_sec=0.1)
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
