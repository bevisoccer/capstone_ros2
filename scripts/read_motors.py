import asyncio
import moteus

async def read():
    for i in [1, 2, 3, 4, 5]:
        c = moteus.Controller(id=i)
        try:
            r = await asyncio.wait_for(c.query(), timeout=2.0)
            pos = r.values[moteus.Register.POSITION]
            print(f"Motor {i}: raw={pos:.4f}  deg={pos*360:.2f}")
        except asyncio.TimeoutError:
            print(f"Motor {i}: TIMEOUT — not on CAN bus")
        except Exception as e:
            print(f"Motor {i}: ERROR — {e}")

asyncio.run(read())
