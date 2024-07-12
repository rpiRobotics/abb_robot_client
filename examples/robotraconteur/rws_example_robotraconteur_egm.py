# Example using the ABB RWS Robot Raconteur Client with EGM
# The Robot Raconteur service must be running and configured to communicate with the robot controller using
# the HTTP RWS server on the robot. The EGM must also be configured to communicate with the computer
# running the Robot Raconteur driver. The networking is configured on the robot configuration in a "reverse socket"
# configuration. The driver by default listens on port 6510 for the EGM connection. The EGM must be configured
# to point to the correct IP address of the computer running the Robot Raconteur driver.
#
# This example uses the Module1.mod and EIO.cfg files found in test/rapid. The client can also be used with
# abb-motion-program-exec to execute motion programs on the robot and use this driver to receive real-time feedback
# and command the EGM.
#
# Having EGM active also populates the robot_state wire members with real-time feedback from the robot. The
# EGM streaming must be activate with the EGMStreamStart or other EGM function to begin streaming data.

from RobotRaconteur.Client import *
import time
import numpy as np
from contextlib import suppress

c = RRN.ConnectService('rr+tcp://localhost:59926?service=robot')
rws_const = RRN.GetConstants("experimental.abb_robot.rws", c)

# Start the EGM using the provided example Module1.mod from test/rapid. If using abb-motion-program-exec
# see the examples in that repository for instructions to enable EGM streaming.
c.setf_analog_io("mode", 0)
# Start and stop tasks
c.resetpp()
task_cycle = rws_const["TaskCycle"]
c.start(task_cycle["once"], ["T_ROB1"])
time.sleep(1)

# Peek the current robot_state. This is the Robot Raconteur standard data structure. 
# Can also connect the wire for real-time streaming
robot_state, _= c.robot_state.PeekInValue()
print(robot_state)

# Peek the ABB specific format EGM feedback data. Can also connect the wire for real-time streaming.
egm_state, _ = c.egm_state.PeekInValue()
print(egm_state)

time.sleep(5)

# Command the robot using EGM position control
egm_joint_command = RRN.GetStructureType("experimental.abb_robot.egm.EGMJointTarget", c)

c.setf_analog_io("mode", 1)
c.resetpp()
c.start(task_cycle["once"], ["T_ROB1"])
time.sleep(2)
cmd = egm_joint_command()
cmd.joints = [21, -10, -20, 30, 40, 50]
cmd.external_joints = [90,100]
cmd.joints_speed = [10,10,10,10,10,10]
cmd.external_joints_speed = [10,10]
cmd.rapid_to_robot = [88,99]
c.egm_joint_command.PokeOutValue(cmd)
time.sleep(3)
cmd2 = egm_joint_command()
cmd2.joints = cmd.joints
cmd2.joints_speed = [0,0,0,0,0,0]
c.egm_joint_command.PokeOutValue(cmd2)
time.sleep(2)
egm_state, _ = c.egm_state.PeekInValue()
print(egm_state)
c.setf_digital_io("stop_egm", 1)
time.sleep(3)