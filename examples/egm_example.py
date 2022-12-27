# Simple example using EGM client
# See https://github.com/rpiRobotics/abb_motion_program_exec/tree/master/examples for more sophisticated examples

# This example requires a robot with EGM configured. The following module can be used for EGM joint
# command: https://github.com/rpiRobotics/EGM_Toolbox/blob/main/egm/egm.mod

from abb_robot_client.egm import EGM
import time
import math

# Create EGM
egm = EGM()

# Wait for valid data
while True:
    res, state = egm.receive_from_robot(timeout=0.5)
    if res:
        break

initial_joints = state.joint_angles
j = initial_joints.copy()

t1 = time.perf_counter()

# Demonstrate moving Joint 6
for i in range(10000):
    res, state = egm.receive_from_robot(timeout=0.1)
    if not res:
        raise Exception("Robot communication lost")
    t2 = time.perf_counter()
    j[5] = math.sin(t2-t1)*15 + initial_joints[5]
    print(f"sending command {j}")
    egm.send_to_robot(j)

    
    


