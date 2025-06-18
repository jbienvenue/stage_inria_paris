#!/usr/bin/python3 -B

# Copyright 2022 mjbots Robotic Systems, LLC.  info@mjbots.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
This example shows how to use Controller.set_position_wait_complete to
execute a position mode command and wait for its completion.
"""

import asyncio
import moteus

c = moteus.Controller(id=6)
async def stop_all():
    await c.set_stop()

async def main():

    # Clear any outstanding faults.
    start = await c.set_stop(query=True)
    null = start.values[1]
    while True:
        # This will periodically command and poll the controller until
        # the target position achieves the commanded value.
        result = await c.set_position_wait_complete(
            position=null+0.25, accel_limit=2.0)
        print(result)
        print(result.values[1])
        # Then go back to zero, and eventually try again.
        result = await c.set_position_wait_complete(
            position=null-0.25, accel_limit=2.0)
        print(result)
        print(result.values[1])


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except:
        asyncio.run(stop_all())