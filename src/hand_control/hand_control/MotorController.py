import moteus
import math
import asyncio
import numpy as np

# =============================================================================
# Robot Arm Control Code (Motor Control)
# =============================================================================
class MoteusMotorController:
    def __init__(self, motor_ids, angle_limits=None):
        """
        Initialize moteus controllers for each motor.
        
        Parameters:
            motor_ids: List of motor IDs.
            angle_limits: Optional dict mapping motor_id to (min_angle, max_angle).
                         Defaults to (-360, 360) for each motor.
        """
        self.controllers = {motor_id: moteus.Controller(id=motor_id) for motor_id in motor_ids}
        self.angle_limits = angle_limits if angle_limits is not None else {motor_id: (-360, 360) for motor_id in motor_ids}
        self.zero_offsets = {}  # Zero offsets (to be calibrated at startup)

    def angle_to_motor_pos(self, angle):
        """Convert an angle (in degrees) to a motor position value."""
        return angle / 360

    async def get_raw_motor_positions(self, motor_id):
        """Query a specific motor and return its raw position."""
        controller = self.controllers[motor_id]
        result = await controller.query()
        pos = result.values.get(moteus.Register.POSITION)
        if pos is None:
            print(f"[WARN] Motor {motor_id} did not return POSITION. Trying ABS_POSITION...")
            pos = result.values.get(moteus.Register.ABS_POSITION, 0)
        return pos

    async def get_all_motor_positions(self):
        """Obtain the positions for all motors."""
        try:
            positions = [
                2 * np.pi * (await self.get_raw_motor_positions(motor_id) - self.zero_offsets[motor_id])
                for motor_id in self.controllers
            ]
        except Exception as e:
            print(f"[ERROR] Could not read positions: {e}")
            positions = []
        return positions

    async def calibrate_zero_offsets(self):
        """Calibrate zero offsets for each motor by reading current positions."""
        print("[INFO] Calibrating zero offsets from current motor positions... 🔧")
        for motor_id in self.controllers:
            try:
                measurement = await self.get_raw_motor_positions(motor_id)
                print(f"[INFO] Motor {motor_id} current position measurement: {measurement:.4f}")
                self.zero_offsets[motor_id] = measurement
                print(f"[INFO] Motor {motor_id} zero offset set to {measurement:.4f}.")
            except Exception as e:
                print(f"[ERROR] Could not read position for motor {motor_id}: {e}. Defaulting offset to 0.")
                self.zero_offsets[motor_id] = 0

    async def set_motor_position(self, motor_id, position, velocity_limit=0.05, accel_limit=1, torque_limit=20):
        """Set motor position along with velocity, acceleration, and torque limits."""
        if motor_id not in self.controllers:
            raise ValueError(f"Motor with ID {motor_id} not initialized.")
        controller = self.controllers[motor_id]
        try:
            await controller.set_position(
                position=position,
                velocity_limit=velocity_limit,
                accel_limit=accel_limit,
                maximum_torque=torque_limit,
                watchdog_timeout=math.nan,
            )
        except Exception as e:
            print(f"[ERROR] Failed to set position for motor {motor_id}: {e}")
            raise

    async def lock_motor(self, motor_id):
        """
        Lock a specific motor (set all command parameters to NaN).
        This is equivalent to "d pos nan 0 nan" in tview.
        """
        if motor_id not in self.controllers:
            raise ValueError(f"Motor with ID {motor_id} not initialized.")
        controller = self.controllers[motor_id]
        try:
            await controller.set_position(
                position=math.nan,
                accel_limit=math.nan,
                maximum_torque=math.nan,
                watchdog_timeout=math.nan,
            )
        except Exception as e:
            print(f"[ERROR] Failed to lock motor {motor_id}: {e}")
            raise

    async def lock_all(self):
        """Stop all motors concurrently by locking them."""
        print("[INFO] Stopping all motors.")
        tasks = [self.lock_motor(motor_id) for motor_id in self.controllers]
        await asyncio.gather(*tasks)

    async def set_motor_angle(self, motor_id, angle, velocity_limit=0.05, accel_limit=1, torque_limit=20):
        """
        Set a motor's position based on an angle command (in degrees).
        The angle is clamped to defined limits and adjusted by the zero offset.
        """
        if motor_id not in self.controllers:
            raise ValueError(f"Motor with ID {motor_id} not initialized.")
        # Clamp the angle to the allowed limits.
        min_angle, max_angle = self.angle_limits.get(motor_id, (-360, 360))
        if angle < min_angle:
            print(f"[WARNING] Angle {angle}° is below minimum limit for motor {motor_id} ({min_angle}°). Clamping.")
            angle = min_angle
        elif angle > max_angle:
            print(f"[WARNING] Angle {angle}° is above maximum limit for motor {motor_id} ({max_angle}°). Clamping.")
            angle = max_angle
        
        motor_delta = self.angle_to_motor_pos(angle)
        zero_offset = self.zero_offsets.get(motor_id, 0)
        motor_pos = zero_offset + motor_delta
        await self.set_motor_position(motor_id, motor_pos, velocity_limit, accel_limit, torque_limit)

    async def stop_motor(self, motor_id):
        """Stop a specific motor."""
        if motor_id not in self.controllers:
            raise ValueError(f"Motor with ID {motor_id} not initialized.")
        try:
            await self.controllers[motor_id].set_stop()
        except Exception as e:
            print(f"[ERROR] Failed to stop motor {motor_id}: {e}")

    async def stop_all(self):
        """Stop all motors concurrently."""
        print("[INFO] Stopping all motors.")
        tasks = [self.stop_motor(motor_id) for motor_id in self.controllers]
        await asyncio.gather(*tasks)