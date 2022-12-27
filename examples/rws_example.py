# Simple example using RWS client
# See https://github.com/rpiRobotics/abb_motion_program_exec/tree/master/examples for more sophisticated examples

from abb_robot_client.rws import RWS, SubscriptionResourceRequest, SubscriptionResourceType, SubscriptionResourcePriority

import time

c = RWS()

print(c.get_rapid_variable("tool0"))

print(c.get_rapid_variables(None))

print(c.get_jointtarget())
print(c.get_robtarget())

print(c.try_create_ipc_queue("my_queue"))
print(c.try_create_ipc_queue("my_send_queue"))

c.send_ipc_message("my_queue", "Test message!", "my_send_queue")
c.send_ipc_message("my_queue", "Test message 2!", "my_send_queue")
c.send_ipc_message("my_queue", "Test message 3!", "my_send_queue")
r = c.read_ipc_message("my_queue")
print(r)

def handler(m):
    print(m)

sub = c.subscribe([
    SubscriptionResourceRequest(SubscriptionResourceType.ControllerState, SubscriptionResourcePriority.Medium),
    SubscriptionResourceRequest(SubscriptionResourceType.OperationalMode, SubscriptionResourcePriority.Medium),
    SubscriptionResourceRequest(SubscriptionResourceType.ExecutionState, SubscriptionResourcePriority.Medium),
    SubscriptionResourceRequest(SubscriptionResourceType.Elog, SubscriptionResourcePriority.Medium),
    SubscriptionResourceRequest(SubscriptionResourceType.PersVar, SubscriptionResourcePriority.Medium, 
        {"task": None, "name": "motion_program_state"}),
    SubscriptionResourceRequest(SubscriptionResourceType.Signal, SubscriptionResourcePriority.High, "motion_program_current_cmd_num"),
    SubscriptionResourceRequest(SubscriptionResourceType.IpcQueue, SubscriptionResourcePriority.Medium, "my_queue"),
    ], handler)
time.sleep(0.5)
c.send_ipc_message("my_queue", "Test message 3!", "my_send_queue")
time.sleep(2)

