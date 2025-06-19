import asyncio, moteus, copy
import matplotlib.pyplot as plt
import time
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
    velocities_engine = []
    velocities_stop = []
    accelerations_stop = []
    accelerations_engine = []
    if STOP_ENGINE:
        await stop()
        Q = await c.query()
        Pause = asyncio.sleep(pause)
        cur = Q.values[P]
        velocities_stop.append(Q.values[V])
        positions_stop.append(cur)
        startT = time.time()
        while not correct(cur, start):
            await Pause
            Q = await c.query()
            Pause = asyncio.sleep(pause)
            cur = Q.values[P]
            positions_stop.append(cur)
            velocities_stop.append(Q.values[V])
            endT = time.time()
            T = endT-startT
            accelerations_stop.append((velocities_stop[-1]-velocities_stop[-2])/T)
            startT = endT
        last = cur
    startT = None
    lastV = None
    while True:
        await Pause
        res = await c.set_position(position=pos, maximum_torque=0.02, accel_limit=2.0, query=True)
        Pause = asyncio.sleep(pause)
        count = max(count-1, 0)
        cur = res.values[P]
        positions_engine.append(cur)
        velocities_engine.append(res.values[V])
        endT = time.time()
        if startT is not None:
            T = endT-startT
            accelerations_engine.append((velocities_engine[-1]-lastV)/T)
        startT = endT
        lastV = velocities_engine[-1]
        if not correct(cur, last) or pos-eps < cur < pos+eps:
            await Pause
            return cur, positions_engine, positions_stop, velocities_engine, velocities_stop, accelerations_engine, accelerations_stop
        last = cur

async def main():
    result = await c.set_stop(query=True)
    start = result.values[1]
    goalR = start+0.5
    goalL = start-0.5
    last = start
    position_groups = []
    velocities_groups = []
    accelerations_groups = []
    curGoal, nextGoal = goalR, goalL
    curFunc, nextFunc = (lambda a,b:a>b), (lambda a,b:a<b)
    while True:
        last, positions_engine, positions_stop, velocities_engine, velocities_stop, accelerations_engine, accelerations_stop = await go_abandon(curGoal, last, curFunc, start)
        position_groups.append(positions_stop)
        position_groups.append(positions_engine)
        velocities_groups.append(velocities_stop)
        velocities_groups.append(velocities_engine)
        accelerations_groups.append(accelerations_stop)
        accelerations_groups.append(accelerations_engine)
        if curGoal-eps < last < curGoal+eps:break
        curGoal, nextGoal = nextGoal, curGoal
        curFunc, nextFunc = nextFunc, curFunc
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
    for n, data in enumerate(accelerations_groups):
        plt.plot(range(start, start+len(data)), data, c=['green', 'red', 'blue', 'orange'][n%4])
        start += len(data)
    #plt.plot(range(len(E)-1), [(E[n+1]-E[n])/pause for n in range(len(E)-1)])
    plt.show(block=True)
    

try:
    asyncio.run(main())
except:
    asyncio.run(stop())
    raise RuntimeError()