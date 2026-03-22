import asyncio
import moteus

async def read():
    for i in [1, 2, 3, 4]:
        c = moteus.Controller(id=i)
        r = await c.query()
        pos = r.values[moteus.Register.POSITION]
        print(f"Motor {i}: raw={pos:.4f}  deg={pos*360:.2f}")

asyncio.run(read())
