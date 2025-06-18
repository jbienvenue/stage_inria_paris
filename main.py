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
    positions_engine = []
    positions_stop = []
    await c.set_stop()
    cur = (await c.query()).values[1]
    positions_stop.append(cur)
    while not correct(cur, start):
        cur = (await c.query()).values[1]
        positions_stop.append(cur)
    last = cur
    while True:
        res = await c.set_position(position=pos, maximum_torque=0.025, accel_limit=9.8, query=True)
        count = max(count-1, 0)
        cur = res.values[1]
        positions_engine.append(cur)
        if not correct(cur, last) and correct(cur, start+(start-Flast)):
            return cur, res, positions_engine, positions_stop
        if pos-eps < cur < pos+eps:
            return cur, res, positions_engine, positions_stop
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
    position_groups = []
    while True:
        last, final, positions_engine, positions_stop = await go_abandon(goalR, last, lambda a,b:a>b, start)
        #print(final, goalR)
        position_groups.append(positions_stop)
        position_groups.append(positions_engine)
        if goalR-eps < last < goalR+eps:break
        last, final, positions_engine, positions_stop = await go_abandon(goalL, last, lambda a,b:a<b, start)
        #print(final, goalL)
        position_groups.append(positions_stop)
        position_groups.append(positions_engine)
        if goalL-eps < last < goalL+eps:break
    await stop()
    plt.ion()
    start = 0
    for n, data in enumerate(position_groups):
        plt.plot(range(start, start+len(data)), data, c=['green', 'red', 'green', 'orange'][n%4])
        start += len(data)
    plt.show(block=True)
    #print(position_groups)
    

try:
    asyncio.run(main())
except:
    asyncio.run(stop())
    raise RuntimeError()