import asyncio
import moteus
import sys

async def set_motor(motor_id, position):
    c = moteus.Controller(id=motor_id)
    for _ in range(50):
        await c.set_position(
            position=position,
            velocity_limit=0.05,
            accel_limit=0.03,
            maximum_torque=6.0,
            watchdog_timeout=0.5
        )
        await asyncio.sleep(0.05)
    r = await c.query()
    print(f"Motor {motor_id} now at: {r.values[moteus.Register.POSITION]:.4f}")

motor_id = int(sys.argv[1])
position = float(sys.argv[2])
asyncio.run(set_motor(motor_id, position))
