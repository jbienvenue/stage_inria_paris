import asyncio, moteus, copy
import time
import math
c = moteus.Controller(id=6)
eps = 0.01
STOP_ENGINE = True
V = moteus.Register.VELOCITY
P = moteus.Register.POSITION
Torq = moteus.Register.TORQUE
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

def acceleration(pos):
    return -2*math.pi*math.sin(2*math.pi*pos)*g

def corrected_speed(pos, startPos):
    return speed_angle(pos-startPos)/0.5

async def try_go_handle(pos, correct, cur, startPos):
    vel = None
    while True:
        last = cur
        if vel:
            max_vel = corrected_speed(cur+vel*pause, startPos)
        else:
            max_vel = corrected_speed(cur, startPos)
        res = await c.set_position(position=pos, maximum_torque=max_torque, velocity_limit=max_vel, kp_scale=1.5, query=True)
        cur = res.values[P]
        vel = res.values[V]
        if not correct(cur, last):
            rep = False, cur
            break
        if pos-eps < cur < pos+eps:
            rep = True, cur
            break
        await asyncio.sleep(pause)
    return rep

async def go_abandon(pos, Flast, correct, startPos):
    query_override = copy.deepcopy(c.query_resolution)
    query_override.trajectory_complete = moteus.multiplex.INT8
    last = Flast
    if STOP_ENGINE:
        await stop()
        Q = await c.query()
        Pause = asyncio.create_task(asyncio.sleep(pause))
        cur = Q.values[P]
        while not correct(cur, startPos):
            await Pause
            Q = await c.query()
            Pause = asyncio.sleep(pause)
            cur = Q.values[P]
        last = cur
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
        lastV = vel
        is_finish = False
        if abs(vel) > corrected_speed(cur, startPos):
            res = await try_go_handle(pos, correct, cur, startPos)
            if res[0]:
                is_finish = True
                cur = res[1]
        if not correct(cur, last) or pos-eps < cur < pos+eps or is_finish:
            await Pause
            return cur
        last = cur

async def main():
    result = await c.set_stop(query=True)
    startPos = result.values[1]
    goalR = startPos+0.5
    goalL = startPos-0.5
    last = startPos
    curGoal, nextGoal = goalR, goalL
    curFunc, nextFunc = (lambda a,b:a>b), (lambda a,b:a<b)
    while True:
        while True:
            last = await go_abandon(curGoal, last, curFunc, startPos)
            if curGoal-eps < last < curGoal+eps:break
            curGoal, nextGoal = nextGoal, curGoal
            curFunc, nextFunc = nextFunc, curFunc
        count_a_row = 0
        while True:
            res = await c.set_position(position=curGoal, maximum_torque=max_torque, kp_scale=1.5, query=True)
            await asyncio.sleep(pause)
            cur = res.values[P]
            if not (curGoal-eps < cur < curGoal+eps):
                count_a_row += 1
            else:
                count_a_row = 0
            if count_a_row > 50:
                break
        turn = True
        while turn:
            turn = False
            if curGoal > startPos:
                turn = cur > curGoal+eps
                delta = +1
            else:
                turn = cur < curGoal-eps
                delta = -1
            if turn:
                startPos += delta
                curGoal += delta
                nextGoal += delta

try:
    asyncio.run(main())
except:
    asyncio.run(stop())
    raise RuntimeError()