import asyncio, moteus, copy
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
        res = await c.set_position(position=pos, maximum_torque=0.1, accel_limit=2.0, velocity_limit=5, query=True, query_override=query_override)
        #nres = await c.query()
        #print(res)
        count = max(count-1, 0)
        cur = res.values[1]
        is_complete = res.values[moteus.Register.TRAJECTORY_COMPLETE]
        #print(res)
        if count == 0 and not correct(cur, last) and correct(cur, start+(start-Flast)) and not is_complete:
            print(L)
            return False, cur, res
        if count == 0 and is_complete:
            print(L)
            return True, cur, res
        last = cur
        L.append(cur)
        await asyncio.sleep(0.02)
            

async def main():
    result = await c.set_stop(query=True)
    start = result.values[1]
    print(await c.set_recapture_position_velocity(query=True))
    print(start)
    goalR = start+0.5
    goalL = start-0.5
    last = start
    while True:
        is_ok, last, final = await go_abandon(goalR, last, lambda a,b:a>b, start)
        print("#"*30)
        print(final)
        if is_ok:break
        is_ok, last, final = await go_abandon(goalL, last, lambda a,b:a<b, start)
        print("#"*30)
        print(final)
        if is_ok:break
    await stop()
    

try:
    asyncio.run(main())
except:
    asyncio.run(stop())
    raise RuntimeError()