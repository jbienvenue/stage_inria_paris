import asyncio, moteus, copy
import matplotlib.pyplot as plt
c = moteus.Controller(id=6)
eps = 0.01
async def stop():
    await c.set_stop(query=True)

async def go_abandon(pos, Flast, correct, start):
    query_override = copy.deepcopy(c.query_resolution)
    query_override.trajectory_complete = moteus.multiplex.INT8
    count = 2
    last = Flast
    L = []
    await c.set_stop()
    cur = (await c.query()).values[1]
    while not correct(cur, start):
        cur = (await c.query()).values[1]
    last = cur
    while True:
        res = await c.set_position(position=pos, maximum_torque=0.025, accel_limit=9.8, query=True)
        print(res)
        count = max(count-1, 0)
        cur = res.values[1]
        L.append(cur)
        if not correct(cur, last) and correct(cur, start+(start-Flast)):
            return cur, res, L
        if pos-eps < cur < pos+eps:
            return cur, res, L
        last = cur
        await asyncio.sleep(0.02)

async def main():
    result = await c.set_stop(query=True)
    start = result.values[1]
    print(await c.set_recapture_position_velocity(query=True))
    print(start)
    goalR = start+0.5
    goalL = start-0.5
    last = start
    M = []
    while True:
        last, final, L = await go_abandon(goalR, last, lambda a,b:a>b, start)
        print(final, goalR)
        M.append(L)
        if goalR-eps < last < goalR+eps:break
        last, final, L = await go_abandon(goalL, last, lambda a,b:a<b, start)
        print(final, goalL)
        M.append(L)
        if goalL-eps < last < goalL+eps:break
    await stop()
    plt.ion()
    start = 0
    for n, data in enumerate(M):
        plt.plot(range(start, start+len(data)), data, c='red' if n%2 else 'green')
        start += len(data)
    plt.show(block=True)
    print(M)
    

try:
    asyncio.run(main())
except:
    asyncio.run(stop())
    raise RuntimeError()