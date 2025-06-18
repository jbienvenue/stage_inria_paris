import asyncio, moteus, copy
import matplotlib.pyplot as plt
c = moteus.Controller(id=6)
eps = 0.01
STOP_ENGINE = True
V = moteus.Register.VELOCITY
P = moteus.Register.POSITION
pause = 0.02
async def stop():
    await c.set_stop(query=True)

async def go_abandon(pos, Flast, correct, start):
    query_override = copy.deepcopy(c.query_resolution)
    query_override.trajectory_complete = moteus.multiplex.INT8
    count = 2
    last = Flast
    positions_engine = []
    positions_stop = []
    velocities = []
    if STOP_ENGINE:
        await c.set_stop()
        Q = await c.query()
        cur = Q.values[P]
        velocities.append(Q.values[V])
        positions_stop.append(cur)
        while not correct(cur, start):
            Q = await c.query()
            cur = Q.values[P]
            positions_stop.append(cur)
            velocities.append(Q.values[V])
        last = cur
    while True:
        res = await c.set_position(position=pos, maximum_torque=0.02, accel_limit=(9.8 if not correct(cur, start) else 2.0), query=True)
        count = max(count-1, 0)
        cur = res.values[P]
        positions_engine.append(cur)
        velocities.append(res.values[V])
        if not correct(cur, last):# and correct(cur, start+(start-Flast)):
            return cur, res, positions_engine, positions_stop, velocities
        if pos-eps < cur < pos+eps:
            return cur, res, positions_engine, positions_stop, velocities
        last = cur
        await asyncio.sleep(pause)

async def main():
    result = await c.set_stop(query=True)
    start = result.values[1]
    goalR = start+0.5
    goalL = start-0.5
    last = start
    position_groups = []
    velocities_groups = []
    while True:
        last, final, positions_engine, positions_stop, velocities = await go_abandon(goalR, last, lambda a,b:a>b, start)
        #print(final, goalR)
        position_groups.append(positions_stop)
        position_groups.append(positions_engine)
        velocities_groups.append(velocities)
        if goalR-eps < last < goalR+eps:break
        last, final, positions_engine, positions_stop, velocities = await go_abandon(goalL, last, lambda a,b:a<b, start)
        #print(final, goalL)
        position_groups.append(positions_stop)
        position_groups.append(positions_engine)
        velocities_groups.append(velocities)
        if goalL-eps < last < goalL+eps:break
    await stop()
    plt.ion()
    start = 0
    for n, data in enumerate(position_groups):
        plt.plot(range(start, start+len(data)), data, c=['green', 'red'][n%2])
        start += len(data)
    plt.show(block=True)
    start = 0
    for n, data in enumerate(velocities_groups):
        plt.plot(range(start, start+len(data)), data, c=['green', 'red'][n%2])
        start += len(data)
    plt.show(block=True)
    start = 0
    E = sum(velocities_groups, start=[])
    Y = []
    for n in range(len(E)-1):
        Y.append((E[n+1]-E[n])/pause)
    plt.plot(range(len(E)-1), Y)
    plt.show(block=True)
    #print(position_groups)
    

try:
    asyncio.run(main())
except:
    asyncio.run(stop())
    raise RuntimeError()