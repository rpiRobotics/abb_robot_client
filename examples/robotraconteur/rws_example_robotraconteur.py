# Example using the ABB RWS Robot Raconteur Client
# The Robot Raconteur service must be running and configured to communicate with the robot controller using
# the HTTP RWS server on the robot. This example uses the Module1.mod and EIO.cfg files found in test/rapid

# This example does not use EGM. EGM is required to use robot_state and the EGM functions

from RobotRaconteur.Client import *
import time

c = RRN.ConnectService('rr+tcp://localhost:59926?service=robot')
rws_const = RRN.GetConstants("experimental.abb_robot.rws", c)
task_cycle = rws_const["TaskCycle"]

# Modify RAPID variables
print(c.setf_rapid_variable("test_var_str", "T_ROB1", "\"Hello, World!\""))
print(c.getf_rapid_variable("test_var_str", "T_ROB1"))

print(c.setf_rapid_variable("test_var_num", "T_ROB1", "1123.456"))
print(c.getf_rapid_variable("test_var_num", "T_ROB1"))

# Read the controller state
print(c.getf_execution_state())
print(c.getf_controller_state())
print(c.getf_operation_mode())

print(c.operational_mode)
print(c.controller_state)

# Get and set signals
c.setf_digital_io("test_digital_io1", 1)
print(c.getf_digital_io("test_digital_io1"))
c.setf_analog_io("test_analog_io1", 14.285)
print(c.getf_analog_io("test_analog_io1"))

# Start and stop tasks
c.resetpp()
task_cycle = rws_const["TaskCycle"]
c.start(task_cycle["once"], ["T_ROB1"])
time.sleep(1)
c.stop()

# Activate and deactivate tasks
c.deactivate_task("T_ROB1")
c.activate_task("T_ROB1")

# Read event log
print(c.read_event_log())

# Read current robot position (slow rate)
print(c.getf_jointtarget("ROB_1"))
print(c.getf_robtarget("ROB_1"))
print(c.getf_robtarget2("ROB_1", "tool0", "wobj0", "Base"))

# Change the "speed ratio" to slow down motions
c.speed_ratio = 0.1
time.sleep(1)
c.speed_ratio = 1.0

# See rws_example_robotraconteur_egm.py for examples using EGM for real-time feedback and control
