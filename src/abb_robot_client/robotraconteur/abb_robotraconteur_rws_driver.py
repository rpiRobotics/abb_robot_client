import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from abb_robot_client import egm, rws
import traceback
import threading
import time
import numpy as np
from robotraconteur_abstract_robot import AbstractRobot
from contextlib import suppress
import argparse
from RobotRaconteurCompanion.Util.RobDef import register_service_types_from_resources
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
import drekar_launch_process
import general_robotics_toolbox as rox

class ABBRWSRobotImpl(AbstractRobot):
    def __init__(self, robot_info, robot_url):
        super().__init__(robot_info, 6)

        self._robot_url = robot_url

        self._base_set_controller_state = False
        self._base_set_operational_mode = False
        self.robot_info.robot_capabilities = 0

        self._uses_homing = False
        self._has_position_command = False
        self._has_velocity_command = False
        self._has_jog_command = False

        self._rws = None
        self._egm = None
        self._missed_egm = 0

        self._abb_rws_const = self._node.GetConstants("experimental.abb_robot.rws")
        self._task_cycle = self._abb_rws_const["TaskCycle"]
        self._task_execution_state = self._abb_rws_const["TaskExecutionState"]

        self._rapid_exec_state = self._node.GetStructureType("experimental.abb_robot.rws.RAPIDExecutionState")
        self._task_state_type = self._node.GetStructureType("experimental.abb_robot.rws.TaskState")

        self._event_log_entry_type = self._node.GetStructureType("experimental.abb_robot.rws.EventLogEntry")

        self._jointtarget_type = self._node.GetStructureType("experimental.abb_robot.rws.JointTarget")
        self._robtarget_type = self._node.GetStructureType("experimental.abb_robot.rws.RobTarget")

        self._confdata_dtype = self._node.GetNamedArrayDType("experimental.abb_robot.rws.ConfData")

        self._egm_robot_type = self._node.GetStructureType("experimental.abb_robot.egm.EGMRobot")
        self._egm_collision_info_type = self._node.GetStructureType("experimental.abb_robot.egm.EGMCollisionInfo")
        # self._timespec2_dtype = self._node.GetNamedArrayDType("com.robotraconteur.datetime.TimeSpec2")

    def RRServiceObjectInit(self, context, service_path):
        super().RRServiceObjectInit(context, service_path)

        self.egm_joint_command.InValueChanged += self._egm_joint_command_invalue_changed
        self.egm_pose_command.InValueChanged += self._egm_pose_command_invalue_changed
        self.egm_path_correction_command.InValueChanged += self._egm_correction_command_invalue_changed

        self._broadcast_downsampler.AddWireBroadcaster(self.egm_state)


    def _start_robot(self):
        self._egm = egm.EGM()
        self._rws = rws.RWS(self._robot_url)

        super()._start_robot()

    def _stop_robot(self):
        super()._stop_robot()

    def _close(self):
        super()._close()

    def _send_robot_command(self, now, joint_pos_cmd, joint_vel_cmd):
        pass

    def _verify_robot_state(self, now):
        self._command_mode = self._robot_command_mode["halt"]
        return True
    
    def _fill_robot_command(self, now):
        return False, None, None
    
    def async_disable(self, handler):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    async def _send_disable(self, handler):
        raise NotImplemented()

    def async_enable(self, handler):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")

    async def _send_enable(self, handler):
        raise NotImplemented()

    def async_reset_errors(self, handler):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def _send_reset_errors(self, handler):
        raise NotImplemented()
    
    def async_jog_freespace(self, joint_position, max_velocity, wait, handler):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def async_jog_joint(self, joint_velocity, timeout, wait, handler):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def execute_trajectory(self, trajectory):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def jog_cartesian(self, velocity, timeout, wait):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def async_home(self, handler):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def tool_attached(self, chain, tool):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")

    def tool_detached(self, chain, tool_name):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")

    def payload_attached(self, chain, payload, pose):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")

    def payload_detached(self, chain, payload_name):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    @AbstractRobot.command_mode.setter
    def command_mode(self, value):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def _run_timestep(self, now):
        res = True
        robot_state = None
        while res:
            res, robot_state1 = self._egm.receive_from_robot()
            if res:                
                robot_state = robot_state1
                self._missed_egm = 0
        if robot_state is None:
            self._missed_egm += 1

        if robot_state is not None:

            self._send_egm_state(robot_state)

            egm_last_recv = self._stopwatch_ellapsed_s()
            self._last_joint_state = egm_last_recv
            self._last_endpoint_state = egm_last_recv
            self._last_robot_state = egm_last_recv
            self._enabled = robot_state.motors_on
            self._ready = self._enabled #robot_state.rapid_running
        
            self._joint_position = np.deg2rad(robot_state.joint_angles)            
            self._endpoint_pose = self._node.ArrayToNamedArray(\
                np.concatenate((robot_state.cartesian[1],robot_state.cartesian[0]*1e-3)), self._pose_dtype)
            
        else:
            if self._communication_failure:
                self._joint_position = np.zeros((0,))        
                self._endpoint_pose = np.zeros((0,),dtype=self._pose_dtype)

        if self._error:
            self._ready = False

        super()._run_timestep(now)

    def _send_egm_state(self, robot_state):

        if not self._wires_ready:
            return

        s = self._egm_robot_type()
        robot_message = robot_state.robot_message
        if robot_message.HasField("header"):
            s.seqno = robot_message.header.seqno
            s.tm = robot_message.header.tm
        if robot_state.joint_angles is not None:
            s.joint_position = robot_state.joint_angles
        if robot_state.cartesian is not None:
            s.cartesian_position = self._node.ArrayToNamedArray(\
                np.concatenate((robot_state.cartesian[1],robot_state.cartesian[0]*1e-3)), self._pose_dtype)
        if robot_state.external_axes is not None:
            s.external_joint_position = robot_state.external_axes
        if robot_state.joint_angles_planned is not None:
            s.joint_position_command = robot_state.joint_angles_planned
        if robot_state.cartesian_planned is not None:
            s.cartesian_position_command = self._node.ArrayToNamedArray(\
                np.concatenate((robot_state.cartesian_planned[1],robot_state.cartesian_planned[0]*1e-3)), self._pose_dtype)
        if robot_state.external_axes_planned is not None:
            s.external_joint_position_command = robot_state.external_axes_planned
        s.motor_state = robot_state.motors_on
        s.rapid_ctrl_exec_state = robot_state.rapid_running
        if robot_message.HasField("utilizationRate"):
            s.utilization_rate = robot_message.utilizationRate
        if robot_state.move_index is not None:
            s.move_index = robot_state.move_index
        if robot_state.rapid_from_robot is not None:
            s.rapid_from_robot = robot_state.rapid_from_robot
        if robot_message.HasField("measuredForce"):
            s.measured_force = list(robot_message.measuredForce.force)

        self.egm_state.OutValue = s
        

    def getf_execution_state(self):
        state = self._rws.get_execution_state()
        ret = self._rapid_exec_state()
        es = state.ctrlexecstate
        if es == "running":
            ret.ctrlexecstate = self._task_execution_state["running"]
        elif es == "stopped":
            ret.ctrlexecstate = self._task_execution_state["stopped"]
        else:
            ret.ctrlexecstate = self._task_execution_state["unknown"]

        c = state.cycle
        if c == "forever":
            ret.cycle = self._task_cycle["forever"]
        elif c == "asis":
            ret.cycle = self._task_cycle["asis"]
        elif c == "once":
            ret.cycle = self._task_cycle["once"]
        elif c == "oncedone":
            ret.cycle = self._task_cycle["oncedone"]
        else:
            ret.cycle = self._task_cycle["unknown"]
        return ret
  
    def getf_controller_state(self):
        state = self._rws.get_controller_state()
        if state == "init":
            return self._robot_controller_state["init"]
        elif state == "motoron":
            return self._robot_controller_state["motor_on"]
        elif state == "motoroff":
            return self._robot_controller_state["motor_off"]
        elif state == "guardstop":
            return self._robot_controller_state["guard_stop"]
        elif state == "emergencystop":
            return self._robot_controller_state["emergency_stop"]
        elif state == "emergystopreset":
            return self._robot_controller_state["emergency_stop_reset"]
        else:
            return self._robot_controller_state["undefined"]
    
    def getf_operation_mode(self):
        mode = self._rws.get_operation_mode()
        if mode == "INIT":
            return self._robot_operational_mode["init"]
        elif mode == "MANR":
            return self._robot_operational_mode["manual_reduced_speed"]
        elif mode == "MANF":
            return self._robot_operational_mode["manual_full_speed"]
        elif mode == "AUTO":
            return self._robot_operational_mode["auto"]
        else:
            return self._robot_operational_mode["undefined"]

    
    @property
    def operational_mode(self):
        return self.getf_operation_mode()
    
    @property
    def controller_state(self):
        return self.getf_controller_state()
    
    def start(self, cycle, tasks):
        rws_cycle = "asis"
        if cycle == self._task_cycle["forever"]:
            rws_cycle = "forever"
        elif cycle == self._task_cycle["asis"]:
            rws_cycle = "asis"
        elif cycle == self._task_cycle["once"]:
            rws_cycle = "once"
        elif cycle == self._task_cycle["oncedone"]:
            rws_cycle = "oncedone"

        self._rws.start(rws_cycle, tasks)

    def stop(self):
        self._rws.stop()

    def resetpp(self):
        self._rws.resetpp()

    def activate_task(self, task):
        self._rws.activate_task(task)

    def deactivate_task(self, task):
        self._rws.deactivate_task(task)

    def getf_tasks(self):
        rws_tasks = self._rws.get_tasks()
        ret = []
        for t in rws_tasks.values():
            ret1 = self._task_state_type()
            ret1.name = t.name
            ret1.type = t.type_
            ret1.taskstate = t.taskstate
            ret1.excstate = self._task_execution_state["unknown"]
            if t.taskstate == "running":
                ret1.taskstate = self._task_execution_state["running"]
            elif t.taskstate == "stopped":
                ret1.taskstate = self._task_execution_state["stopped"]
            ret1.active = t.active
            ret1.motiontask = t.motiontask
            ret.append(ret1)
        return ret

    def getf_digital_io(self, signal_name):
        return self._rws.get_digital_io(signal_name)
    
    def setf_digital_io(self, signal_name, value):
        self._rws.set_digital_io(signal_name, value)

    def getf_digital_io2(self, signal_name, network, unit):
        return self._rws.get_digital_io(signal_name, network, unit)
    
    def setf_digital_io2(self, signal_name, network, unit, value):
        self._rws.set_digital_io(signal_name, value, network, unit)

    def getf_analog_io(self, signal_name):
        return self._rws.get_analog_io(signal_name)
    
    def setf_analog_io(self, signal_name, value):
        self._rws.set_analog_io(signal_name, value)

    def getf_analog_io2(self, signal_name, network, unit):
        return self._rws.get_analog_io(signal_name, network, unit)
    
    def setf_analog_io2(self, signal_name, network, unit, value):
        self._rws.set_analog_io(signal_name, value, network, unit)
    
    def getf_rapid_variables(self, task):
        raise RR.NotImplementedException("Not implemented")
    
    def getf_rapid_variable(self, var, task):
        return self._rws.get_rapid_variable(var, task)
    
    def setf_rapid_variable(self, var, task, value):
        self._rws.set_rapid_variable(var, value, task)

    def getf_ramdisk_path(self):
        return self._rws.get_ramdisk_path()
    
    def read_file(self, filename):
        return self._rws.read_file(filename)
    
    def upload_file(self, filename, data):
        self._rws.upload_file(filename, data.tobytes())

    def delete_file(self, filename):
        self._rws.delete_file(filename)

    def list_files(self, path):
        return self._rws.list_files(path)
    
    def read_event_log(self):
        ret = []
        evts = self._rws.read_event_log()
        for e in evts:
            evt = self._event_log_entry_type()
            evt.seqnum = e.seqnum
            evt.msgtype = e.msgtype
            # evt.tstamp = e.tstamp
            evt.title = e.title
            evt.desc = e.desc
            evt.conseqs = e.conseqs
            evt.causes = e.causes
            evt.actions = e.actions
            ret.append(evt)
        return ret
    
    def _rws_jt_to_rr(self, jt):
        ret = self._jointtarget_type()
        ret.robax = jt.robax
        ret.extax = jt.extax
        return ret
    
    def _rr_jt_to_rws(self, jt):
        ret = rws.JointTarget(jt.robax, jt.extax)
        return ret

    def getf_jointtarget(self, mechunit):
        jt = self._rws.get_jointtarget(mechunit)
        return self._rws_jt_to_rr(jt)
    
    def getf_robtarget(self, mechunit):
        rt = self._rws.get_robtarget(mechunit)
        ret = self._robtarget_type()
        ret.pose = self._geometry_util.rox_transform_to_pose(rox.Transform(rox.q2R(rt.rot), rt.trans))
        ret.robconf = self._node.ArrayToNamedArray(rt.robconf, self._confdata_dtype)
        ret.extax = rt.extax

        return ret
    
    def getf_robtarget2(self, mechunit, tool, wobj, coordinate):
        rt = self._rws.get_robtarget(mechunit, tool, wobj, coordinate)
        ret = self._robtarget_type()
        ret.pose = self._geometry_util.rox_transform_to_pose(rox.Transform(rox.q2R(rt.rot), rt.trans))
        ret.robconf = self._node.ArrayToNamedArray(rt.robconf, self._confdata_dtype)
        ret.extax = rt.extax

        return ret
    
    @property
    def speed_ratio(self):
        return self._rws.get_speedratio()/100.0

    @speed_ratio.setter
    def speed_ratio(self, value):
        self._rws.set_speedratio(int(value*100))

    def getf_rapid_variable_jointtarget(self, var, task):
        jt = self._rws.get_rapid_variable_jointtarget(var, task)
        return self._rws_jt_to_rr(jt)
    
    def setf_rapid_variable_jointtarget(self, var, task, value):
        self._rws.set_rapid_variable_jointtarget(var, self._rr_jt_to_rws(value), task)

    def getf_rapid_variable_jointtarget_array(self, var, task):
        jt = self._rws.get_rapid_variable_jointtarget_array(var, task)
        ret = []
        for j in jt:
            ret.append(self._rws_jt_to_rr(j))
        return ret
    
    def setf_rapid_variable_jointtarget_array(self, var, task, value):
        jt = []
        for j in value:
            jt.append(self._rr_jt_to_rws(j))
        self._rws.set_rapid_variable_jointtarget_array(var, jt, task)

    def getf_rapid_variable_num(self, var, task):
        return self._rws.get_rapid_variable_num(var, task)
    
    def setf_rapid_variable_num(self, var, task, value):
        self._rws.set_rapid_variable_num(var, value, task)

    def getf_rapid_variable_num_array(self, var, task):
        return self._rws.get_rapid_variable_num_array(var, task)
    
    def setf_rapid_variable_num_array(self, var, task, value):
        self._rws.set_rapid_variable_num_array(var, value, task)

    def request_rmmp(self, timeout):
        self._rws.request_rmmp(timeout)

    def poll_rmmp(self):
        self._rws.poll_rmmp()

    def _egm_joint_command_invalue_changed(self, value, ts, ep):
        try:
            joint_command = value.joints
            speed_command = value.joints_speed
            if len(speed_command) == 0:
                speed_command = None
            external_joints = value.external_joints
            if len(external_joints) == 0:
                external_joints = None
            external_joints_speed = value.external_joints_speed
            if len(external_joints_speed) == 0:
                external_joints_speed = None
            rapid_to_robot = value.rapid_to_robot
            if len(rapid_to_robot) == 0:
                rapid_to_robot = None
            self._egm.send_to_robot(joint_command, speed_command, external_joints, external_joints_speed, rapid_to_robot)
        except:
            traceback.print_exc()

    def _egm_pose_command_invalue_changed(self, value, ts, ep):
        # print(f"egm_pose_command: {value.cartesian}")
        try:
            
            cart_command = self._node.NamedArrayToArray(value.cartesian)[0]
            speed_command = value.cartesian_speed
            if len(speed_command) == 0:
                speed_command = None
            else:
                speed_command1 = self._node.NamedArrayToArray(speed_command)
                speed_command_lin = speed_command1[0][3:6]*1e3
                speed_command_rot = speed_command1[0][0:3]
                speed_command = np.concatenate((speed_command_lin, speed_command_rot))
            external_joints = value.external_joints
            if len(external_joints) == 0:
                external_joints = None
            external_joints_speed = value.external_joints_speed
            if len(external_joints_speed) == 0:
                external_joints_speed = None
            rapid_to_robot = value.rapid_to_robot
            if len(rapid_to_robot) == 0:
                rapid_to_robot = None
            self._egm.send_to_robot_cart(np.array(cart_command[4:7])*1e3, cart_command[0:4],
                                         speed_command, external_joints, external_joints_speed, rapid_to_robot)
        except:
            traceback.print_exc()

    def _egm_correction_command_invalue_changed(self, value, ts, ep):
        # print(f"egm_correction_command: {value.pos}, {value.age}")
        try:
            p = self._node.NamedArrayToArray(value.pos)[0]*1e3
            self._egm.send_to_robot_path_corr(p, value.age)
        except:
            traceback.print_exc()

def main():

    parser = argparse.ArgumentParser(description="ABB RWS/EGM robot driver service for Robot Raconteur")
    parser.add_argument("--robot-info-file", type=argparse.FileType('r'),default=None,required=True,help="Robot info file (required)")
    parser.add_argument("--robot-url", type=str,default="http://127.0.0.1:80",help="Robot Web Services URL")
    parser.add_argument("--robot-name", type=str,default=None,help="Optional device name override")
    
    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)
    register_service_types_from_resources(RRN, __package__, ["experimental.abb_robot.rws", "experimental.abb_robot.egm"])

    with args.robot_info_file:
        robot_info_text = args.robot_info_file.read()

    info_loader = InfoFileLoader(RRN)
    robot_info, robot_ident_fd = info_loader.LoadInfoFileFromString(robot_info_text, "com.robotraconteur.robotics.robot.RobotInfo", "device")

    attributes_util = AttributesUtil(RRN)
    robot_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(robot_info.device_info)

    robot = ABBRWSRobotImpl(robot_info, args.robot_url)
    try:

        robot._start_robot()
        time.sleep(0.5)
        with RR.ServerNodeSetup("experimental.abb_rws_robot.robot",59926):

            service_ctx = RRN.RegisterService("robot","experimental.abb_robot.rws.ABBRWSRobot",robot)
            service_ctx.SetServiceAttributes(robot_attributes)

            print("Press Ctrl-C to exit")
            drekar_launch_process.wait_exit()
            robot._close()
    except:
        with suppress(Exception):
            robot._close()
        raise

    