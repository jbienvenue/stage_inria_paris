import asyncio, moteus, copy
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
import time
import math
c = moteus.Controller(id=6)
eps = 0.01
STOP_ENGINE = True
V = moteus.Register.VELOCITY
P = moteus.Register.POSITION
Torq = moteus.Register.TORQUE
pause = 0.02
max_torque = 0.05

@dataclass(init=True, repr=True)
class Plot:
    positions:list[float] = field(default_factory=list)
    torques:list[float] = field(default_factory=list)
    velocities:list[float] = field(default_factory=list)
    accelerations:list[float] = field(default_factory=list)
    change:int=0
    mode:bool=True
    last:float=0.0
    def add(self, obj):
        now = time.time()
        self.positions.append(obj.values[P])
        self.torques.append(obj.values[Torq])
        self.velocities.append(obj.values[V])
        if len(self.velocities) > 1:
            self.accelerations.append((self.velocities[-1]-self.velocities[-2])/(now-self.last))
        self.last = now
    
    def plot(self, start, id, color1='green', color2='red', mapping=lambda x:x):
        data = (self.positions, self.torques, self.velocities, self.accelerations)[id]
        data = list(map(mapping, data))
        if id == 3:
            plt.plot(range(start+1, start+self.change), data[:self.change-1], c=color1)
            plt.plot(range(start+self.change, start+len(data)+1), data[self.change-1:], c=color2)
            return start+len(data)+1
        plt.plot(range(start, start+self.change), data[:self.change], c=color1)
        plt.plot(range(start+self.change, start+len(data)), data[self.change:], c=color2)
        return start+len(data)
    
    def plot2d(self, idx, idy, color1='green', color2='red', mapping=lambda x:x):
        dataX = (self.positions, self.torques, self.velocities, self.accelerations)[idx]
        dataY = (self.positions, self.torques, self.velocities, self.accelerations)[idy]
        change = self.change-1
        if len(dataX) > len(dataY):
            dataX = dataX[1:]
        elif len(dataY) > len(dataX):
            dataY = dataY[1:]
        else:
            change += 1
        plt.plot(dataX[:change], dataY[:change], c=color1)
        plt.plot(dataX[change:], dataY[change:], c=color2)

    def change_mode(self):
        assert self.mode
        self.mode = False
        self.change = len(self.positions)

def plot_all(datas, *args, **kwargs):
    start = 0
    for data in datas:
        start = data.plot(start, *args, **kwargs)

def plot2d_all(datas, *args, **kwargs):
    for data in datas:
        data.plot2d(*args, **kwargs)

async def stop():
    await c.set_stop(query=True)

async def handle(pos):
    while True:
        await c.set_position(position=pos, maximum_torque=max_torque, kp_scale=1.5)
g = 1.1
def speed_angle(pos):
    return abs(g*(-math.cos(pos*2*math.pi)+math.cos(math.pi)))

def acceleration(pos):
    return -2*math.pi*math.sin(2*math.pi*pos)*g

def corrected_speed(pos, startPos):
    return speed_angle(pos-startPos)/0.5

async def try_go_handle(pos, correct, cur, startPos, data):
    vel = None
    print('forward try', pos, correct, cur, startPos)
    while True:
        last = cur
        if vel:
            max_vel = corrected_speed(cur+vel*pause, startPos)
        else:
            max_vel = corrected_speed(cur, startPos)
        res = await c.set_position(position=pos, maximum_torque=max_torque, velocity_limit=max_vel, kp_scale=1.5, query=True)
        cur = res.values[P]
        vel = res.values[V]
        data.add(res)
        if not correct(cur, last):return False, cur
        if pos-eps < cur < pos+eps:return True, cur
        await asyncio.sleep(pause)

async def go_abandon(pos, Flast, correct, startPos):
    query_override = copy.deepcopy(c.query_resolution)
    query_override.trajectory_complete = moteus.multiplex.INT8
    data = Plot()
    last = Flast
    if STOP_ENGINE:
        await stop()
        Q = await c.query()
        Pause = asyncio.create_task(asyncio.sleep(pause))
        cur = Q.values[P]
        data.add(Q)
        while not correct(cur, startPos):
            await Pause
            Q = await c.query()
            Pause = asyncio.sleep(pause)
            data.add(Q)
            cur = Q.values[P]
        last = cur
        data.change_mode()
    startT = None
    lastV = None
    while True:
        await Pause
        if lastV:
            max_vel = corrected_speed(last+lastV*pause, startPos)
        else:
            max_vel = corrected_speed(last, startPos)
        res = await c.set_position(position=pos, maximum_torque=max_torque, accel_limit=1.0, velocity_limit=max_vel, query=True)
        Pause = asyncio.create_task(asyncio.sleep(pause))
        vel = res.values[V]
        cur = res.values[P]
        data.add(res)
        lastV = vel
        is_finish = False
        if abs(vel) > corrected_speed(cur, startPos):
            res = await try_go_handle(pos, correct, cur, startPos, data)
            if res[0]:
                is_finish = True
                cur = res[1]
        if not correct(cur, last) or pos-eps < cur < pos+eps or is_finish:
            await Pause
            return cur, data#positions_engine, positions_stop, velocities_engine, velocities_stop, accelerations_engine, accelerations_stop
        last = cur

async def main():
    result = await c.set_stop(query=True)
    startPos = result.values[1]
    goalR = startPos+0.5
    goalL = startPos-0.5
    last = startPos
    datas = []
    curGoal, nextGoal = goalR, goalL
    curFunc, nextFunc = (lambda a,b:a>b), (lambda a,b:a<b)
    while True:
        last, data = await go_abandon(curGoal, last, curFunc, startPos)
        datas.append(data)
        if curGoal-eps < last < curGoal+eps:break
        curGoal, nextGoal = nextGoal, curGoal
        curFunc, nextFunc = nextFunc, curFunc
    print(last, curGoal)
    task = asyncio.create_task(asyncio.to_thread(input, "press enter to go down and finish"))
    while not task.done():
        await c.set_position(position=curGoal, maximum_torque=max_torque, kp_scale=1.5)
        await asyncio.sleep(pause)
    print('go down')
    await c.set_position_wait_complete(position=startPos, velocity_limit=0.1, maximum_torque=0.3, accel_limit=2.0)
    await stop()
    plt.ion()
    plot_all(datas, 0)
    plot_all(datas, 0, color1='black', color2='black', mapping=lambda x:corrected_speed(x, startPos))
    plot_all(datas, 2, mapping=abs)
    plt.show(block=True)
    plot_all(datas, 1)
    plt.show(block=True)
    plot_all(datas, 3)
    plt.show(block=True)
    plot2d_all(datas, 0, 2)
    plt.show(block=True)

try:
    asyncio.run(main())
except:
    asyncio.run(stop())
    raise RuntimeError()