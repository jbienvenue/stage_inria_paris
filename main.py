import asyncio, moteus, copy
import matplotlib.pyplot as plt
c = moteus.Controller(id=6)

async def stop():
    await c.set_stop(query=True)

async def go_abandon(pos, Flast, correct, start):
    query_override = copy.deepcopy(c.query_resolution)
    query_override.trajectory_complete = moteus.multiplex.INT8
    count = 2
    last = Flast
    L = []
    while True:
        res = await c.set_position(position=pos, maximum_torque=0.1, accel_limit=2.0, query=True, query_override=query_override)
        count = max(count-1, 0)
        cur = res.values[1]
        is_complete = res.values[moteus.Register.TRAJECTORY_COMPLETE]
        L.append(cur)
        if count == 0:
            if is_complete:
                return True, cur, res, L
            if not correct(cur, last) and correct(cur, start+(start-Flast)):
                return False, cur, res, L
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
        is_ok, last, final, L = await go_abandon(goalR, last, lambda a,b:a>b, start)
        print(final, goalR)
        M.append(L)
        if is_ok:break
        is_ok, last, final, L = await go_abandon(goalL, last, lambda a,b:a<b, start)
        print(final, goalL)
        M.append(L)
        if is_ok:break
    plt.ion()
    start = 0
    for n, data in enumerate(M):
        plt.plot(range(start, start+len(data)), data, c='red' if n%2 else 'green')
        start += len(data)
    await stop()
    plt.show(block=True)
    print(M)
    

try:
    asyncio.run(main())
except:
    asyncio.run(stop())
    raise RuntimeError()