import asyncio
import moteus
c = moteus.Controller(id=6)
async def stop():
    await c.set_stop()
async def main():
    res = await c.set_stop(query=True)
    startPos = res.values[1]
    while 1:
        await c.set_position(position=startPos+0.5, kp_scale=1.5, maximum_torque=0.3, accel_limit=2.0, velocity_limit=2.0)
        await asyncio.sleep(0.025)

try:
    asyncio.run(main())
except:
    asyncio.run(stop())

