# Simple example using RWS_AIO client
# See https://github.com/rpiRobotics/abb_motion_program_exec/tree/master/examples for more sophisticated examples

from contextlib import suppress
from abb_robot_client.rws_aio import RWS_AIO, SubscriptionResourceRequest, SubscriptionResourceType, \
    SubscriptionResourcePriority

import time
import asyncio

async def main():

    c = RWS_AIO()

    print(await c.get_rapid_variable("tool0"))

    print(await c.get_rapid_variables(None))

    print(await c.get_jointtarget())
    print(await c.get_robtarget())

    print(await c.try_create_ipc_queue("my_queue"))
    print(await c.try_create_ipc_queue("my_send_queue"))

    await c.send_ipc_message("my_queue", "Test message!", "my_send_queue")
    await c.send_ipc_message("my_queue", "Test message 2!", "my_send_queue")
    await c.send_ipc_message("my_queue", "Test message 3!", "my_send_queue")
    r = await c.read_ipc_message("my_queue")
    print(r)

    sub = c.subscribe([
        SubscriptionResourceRequest(SubscriptionResourceType.ControllerState, SubscriptionResourcePriority.Medium),
        SubscriptionResourceRequest(SubscriptionResourceType.OperationalMode, SubscriptionResourcePriority.Medium),
        SubscriptionResourceRequest(SubscriptionResourceType.ExecutionState, SubscriptionResourcePriority.Medium),
        SubscriptionResourceRequest(SubscriptionResourceType.Elog, SubscriptionResourcePriority.Medium),
        SubscriptionResourceRequest(SubscriptionResourceType.PersVar, SubscriptionResourcePriority.Medium, 
            {"task": None, "name": "motion_program_state"}),
        SubscriptionResourceRequest(SubscriptionResourceType.Signal, SubscriptionResourcePriority.High, "motion_program_current_cmd_num"),
        SubscriptionResourceRequest(SubscriptionResourceType.IpcQueue, SubscriptionResourcePriority.Medium, "my_queue"),
        ])

    async def send_ipc():
        await asyncio.sleep(0.5)
        await c.send_ipc_message("my_queue", "Test message 3!", "my_send_queue")
    
    async def recv_sub():
        async for msg in sub:
            print(msg)

    send_ipc_task = asyncio.get_event_loop().create_task(send_ipc())
    recv_sub_task = asyncio.get_event_loop().create_task(recv_sub())

    # Run for a bit
    await asyncio.sleep(2)

    # Shutdown
    recv_sub_task.cancel()

    await send_ipc_task
    with suppress(asyncio.CancelledError):
        await recv_sub_task

asyncio.run(main())
