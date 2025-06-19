import asyncio, moteus, copy
import matplotlib.pyplot as plt
import time
import math
c = moteus.Controller(id=6)
eps = 0.01
STOP_ENGINE = True
V = moteus.Register.VELOCITY
P = moteus.Register.POSITION
pause = 0.02
max_torque = 0.02
async def stop():
    await c.set_stop(query=True)

async def handle(pos):
    while True:
        await c.set_position(position=pos, maximum_torque=max_torque, kp_scale=1.5)
g = 1.1
def speed_angle(pos):
    return abs(g*(-math.cos(pos*2*math.pi)+math.cos(math.pi)))

def corrected_speed(pos, start):
    return speed_angle(pos-start)/0.7

async def go_abandon(pos, Flast, correct, start):
    query_override = copy.deepcopy(c.query_resolution)
    query_override.trajectory_complete = moteus.multiplex.INT8
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
        Pause = asyncio.create_task(asyncio.sleep(pause))
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
        res = await c.set_position(position=pos, maximum_torque=max_torque, accel_limit=1.0, velocity_limit=corrected_speed(last, start), query=True)
        Pause = asyncio.create_task(asyncio.sleep(pause))
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
    startPos = start
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
    #await stop()
    print(last, curGoal)
    task = asyncio.create_task(asyncio.to_thread(input, "press enter to go down and finish"))
    while not task.done():
        await c.set_position(position=curGoal, maximum_torque=max_torque, kp_scale=1.5)
        await asyncio.sleep(pause)
    print('go down')
    await c.set_position_wait_complete(position=startPos, velocity_limit=0.1, maximum_torque=0.3, accel_limit=2.0)
    await stop()
    plt.ion()
    first = False
    for N, datas in enumerate([position_groups, velocities_groups, accelerations_groups]):
        func = (lambda a:a) if N != 1 else abs
        start = 0
        for n, data in enumerate(datas):
            plt.plot(range(start, start+len(data)), list(map(func, data)), c=['green', 'red'][n%2])
            start += len(data)
        if N == 0:
            start = 0
            for n, data in enumerate(datas):
                plt.plot(range(start, start+len(data)), [corrected_speed(i, startPos) for i in data], c='black')
                start += len(data)
        else:
            plt.show(block=True)
        first = True
    

try:
    asyncio.run(main())
except:
    asyncio.run(stop())
    raise RuntimeError()