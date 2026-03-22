import asyncio
import moteus

async def test():
    c = moteus.Controller(id=1)
    
    # Read current
    r = await c.query()
    current = r.values[moteus.Register.POSITION]
    print(f"Motor 1 current: {current:.4f} raw = {current*360:.1f} deg")
    
    print("\nSlowly moving to +0.05 raw (18 deg more)...")
    for _ in range(30):
        await c.set_position(
            position=current + 0.05,
            velocity_limit=0.05,
            accel_limit=0.03,
            maximum_torque=5.0,
            watchdog_timeout=0.5
        )
        await asyncio.sleep(0.1)
    
    r = await c.query()
    print(f"After +0.05: {r.values[moteus.Register.POSITION]:.4f}")
    
    print("Returning to origin...")
    for _ in range(30):
        await c.set_position(
            position=current,
            velocity_limit=0.05,
            accel_limit=0.03,
            maximum_torque=5.0,
            watchdog_timeout=0.5
        )
        await asyncio.sleep(0.1)
    
    print("Moving to -0.05 raw (18 deg less)...")
    for _ in range(30):
        await c.set_position(
            position=current - 0.05,
            velocity_limit=0.05,
            accel_limit=0.03,
            maximum_torque=5.0,
            watchdog_timeout=0.5
        )
        await asyncio.sleep(0.1)
    
    r = await c.query()
    print(f"After -0.05: {r.values[moteus.Register.POSITION]:.4f}")
    
    print("Returning...")
    for _ in range(30):
        await c.set_position(
            position=current,
            velocity_limit=0.05,
            accel_limit=0.03,
            maximum_torque=5.0,
            watchdog_timeout=0.5
        )
        await asyncio.sleep(0.1)
    
    print("Done")

asyncio.run(test())
