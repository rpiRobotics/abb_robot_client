# Copyright 2022 Wason Technology LLC, Rensselaer Polytechnic Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import socket
import select
from ._egm_protobuf import egm_pb2
import traceback
import numpy as np
import errno
from typing import Tuple, NamedTuple, Any

class EGMRobotState(NamedTuple):
    """
    State information returned from robot through EGM
    """
    joint_angles: np.array
    """Joint angles of robot in degrees. Length is 6 or 7 depending on robot"""
    rapid_running: bool
    """True if RAPID program is running on controller"""
    motors_on: bool 
    """True if motors are on"""
    robot_message: Any
    """Full message returned from robot"""
    cartesian: Tuple[np.array,np.array]
    """Cartesian position of robot, in ([x,y,z],[w,x,y,z]) format"""
    joint_angles_planned: np.array
    """Planned (commanded) joint angles of robot in degrees. Length is 6 or 7 depending on robot"""
    cartesian_planned: Tuple[np.array,np.array]
    """Planned (commanded) cartesian position of robot, in ([x,y,z],[w,x,y,z]) format"""
    external_axes: np.array
    """External axes in degrees"""
    external_axes_planned: np.array
    """Planned (commanded) external axes in degrees"""
    measured_force: np.array
    """Measured force wrench in Nm and N"""
    move_index: int
    """Move index of current motion"""
    rapid_from_robot: np.array
    """RAPID variables from robot"""


class EGM(object):
    """
    ABB EGM (Externally Guided Motion) client. EGM provides a real-time streaming connection to the robot using
    UDP, typically at a rate of 250 Hz. The robot controller initiates the connection. The IP address and port of the 
    client must be configured on the robot controller side. The EGM client will send commands to the port it receives
    packets from.

    :param port: The port to receive UDP packets. Defaults to 6510
    """

    def __init__(self, port : int=6510):

        self.socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('',port))
        self.send_sequence_number=0
        self.egm_addr=None
        self.count=0

    def receive_from_robot(self, timeout : float=0) -> Tuple[bool,EGMRobotState]:
        """
        Receive feedback from the robot. Specify an optional timeout. Returns a tuple with success and the current
        robot state.

        :param timeout: Timeout in seconds. May be zero to immediately return if there is no new data.
        :return: Success and robot state as a tuple
        """
        s=self.socket
        s_list=[s]
        try:
            res=select.select(s_list, [], s_list, timeout)
        except select.error as err:
            if err.args[0] == errno.EINTR:
                return False, None
            else:
                raise

        if len(res[0]) == 0 and len(res[2])==0:
            return False, None
        try:
            (buf, addr)=s.recvfrom(65536)
        except:
            self.egm_addr=None
            return False, None

        self.egm_addr=addr

        robot_message=egm_pb2.EgmRobot()
        robot_message.ParseFromString(buf)

        joint_angles=None
        rapid_running=False
        motors_on=False
        cartesian=None
        external_axes= None
        joint_angles_planned = None
        cartesian_planned = None
        external_axes_planned = None
        measured_force = None
        move_index = None
        rapid_from_robot = None

        if robot_message.HasField('feedBack'):
            joints=robot_message.feedBack.joints.joints
            joint_angles=np.array(list(joints))
            if robot_message.feedBack.HasField('cartesian'):
                cart_p = robot_message.feedBack.cartesian.pos
                cart_q = robot_message.feedBack.cartesian.orient
                cartesian = (np.array([cart_p.x,cart_p.y,cart_p.z]),np.array([cart_q.u0,cart_q.u1,cart_q.u2,cart_q.u3]))
            if robot_message.feedBack.HasField('externalJoints'):
                external_axes = np.array(list(robot_message.feedBack.externalJoints.joints))
        if robot_message.HasField('rapidExecState'):
            rapid_running = robot_message.rapidExecState.state == robot_message.rapidExecState.RAPID_RUNNING
        if robot_message.HasField('motorState'):
            motors_on = robot_message.motorState.state == robot_message.motorState.MOTORS_ON
        if robot_message.HasField('planned'):
            planned = robot_message.planned
            if planned.HasField('joints'):
                joints = planned.joints.joints
                joint_angles_planned = np.array(list(joints))
            if planned.HasField('cartesian'):
                cart_p = planned.cartesian.pos
                cart_q = planned.cartesian.orient
                cartesian_planned = (np.array([cart_p.x,cart_p.y,cart_p.z]),np.array([cart_q.u0,cart_q.u1,cart_q.u2,cart_q.u3]))
            if planned.HasField('externalJoints'):
                external_axes_planned = np.array(list(planned.externalJoints.joints))
        if robot_message.HasField('measuredForce'):
            force_active = True
            if robot_message.measuredForce.HasField('fcActive'):
                force_active = robot_message.measuredForce.fcActive
            if force_active:
                force = robot_message.measuredForce.force
                measured_force = np.array(list(force))
        if robot_message.HasField('RAPIDfromRobot'):
            rapid_from_robot = np.array(list(robot_message.RAPIDfromRobot.dnum))
        if robot_message.HasField('moveIndex'):
            move_index = robot_message.moveIndex

        return True, EGMRobotState(
            joint_angles=joint_angles,
            rapid_running=rapid_running,
            motors_on=motors_on,
            robot_message=robot_message,
            cartesian=cartesian,
            joint_angles_planned=joint_angles_planned,
            cartesian_planned=cartesian_planned,
            external_axes=external_axes,
            external_axes_planned=external_axes_planned,
            measured_force=measured_force,
            move_index=move_index,
            rapid_from_robot=rapid_from_robot
        )

    def send_to_robot(self, joint_angles: np.array, speed_ref: np.array = None, external_joints: np.array = None,
                      external_joints_speed: np.array = None, rapid_to_robot: np.array = None) -> bool:
        """
        Send a joint command to robot. Returns False if no data has been received from the robot yet. The EGM
        operation must have been started with EGMActJoint and EGMRunJoint.

        :param joint_angles: Joint angle command in degrees
        :return: True if successful, False if no data received from robot yet
        """

        if not self.egm_addr:
            return False

        self.send_sequence_number+=1

        sensorMessage=egm_pb2.EgmSensor()

        header=sensorMessage.header
        header.mtype=egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_CORRECTION')
        header.seqno=self.send_sequence_number
        self.send_sequence_number+=1

        planned=sensorMessage.planned
        speedRef = sensorMessage.speedRef

        if joint_angles is not None:
            joint_angles2 = list(np.array(joint_angles))
            planned.joints.joints.extend(joint_angles2)

        if speed_ref is not None:
            speed_ref2 = list(np.array(speed_ref))
            speedRef.joints.joints.extend(speed_ref2)

        if external_joints is not None:
            external_joints2 = list(np.array(external_joints))
            planned.externalJoints.joints.extend(external_joints2)

        if external_joints_speed is not None:
            external_joints_speed2 = list(np.array(external_joints_speed))
            speedRef.externalJoints.joints.extend(external_joints_speed2)

        if rapid_to_robot is not None:
            rapid_to_robot2 = list(np.array(rapid_to_robot))
            sensorMessage.RAPIDtoRobot.dnum.extend(rapid_to_robot2)

        buf2=sensorMessage.SerializeToString()

        try:
            self.socket.sendto(buf2, self.egm_addr)
        except:
            return False

        return True

    def send_to_robot_cart(self, pos: np.ndarray, orient: np.ndarray, speed_ref: np.array = None, 
                           external_joints: np.array = None, external_joints_speed: np.array = None, 
                           rapid_to_robot: np.array = None):
        """
        Send a cartesian command to robot. Returns False if no data has been received from the robot yet. The pose
        is relative to the tool, workobject, and frame specified when the EGM operation is initialized. The EGM
        operation must have been started with EGMActPose and EGMRunPose.

        :param pos: The position of the TCP in millimeters [x,y,z]
        :param orient: The orientation of the TCP in quaternions [w,x,y,z]
        :return: True if successful, False if no data received from robot yet
        """
        if not self.egm_addr:
            return False

        self.send_sequence_number+=1

        sensorMessage=egm_pb2.EgmSensor()

        header=sensorMessage.header
        header.mtype=egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_CORRECTION')
        header.seqno=self.send_sequence_number
        self.send_sequence_number+=1

        planned=sensorMessage.planned
        speedRef = sensorMessage.speedRef

        # if joint_angles is not None:
        #     joint_angles2 = list(np.rad2deg(joint_angles))
        #     planned.joints.joints.extend(joint_angles2)

        if pos is not None and orient is not None:
            planned.cartesian.pos.x = pos[0]
            planned.cartesian.pos.y = pos[1]
            planned.cartesian.pos.z = pos[2]
            planned.cartesian.orient.u0 = orient[0]
            planned.cartesian.orient.u1 = orient[1]
            planned.cartesian.orient.u2 = orient[2]
            planned.cartesian.orient.u3 = orient[3]

        if speed_ref is not None:
            speed_ref2 = list(np.array(speed_ref))
            speedRef.cartesians.value.extend(speed_ref2)

        if external_joints is not None:
            external_joints2 = list(np.array(external_joints))
            planned.externalJoints.joints.extend(external_joints2)

        if external_joints_speed is not None:
            external_joints_speed2 = list(np.array(external_joints_speed))
            speedRef.externalJoints.joints.extend(external_joints_speed2)

        if rapid_to_robot is not None:
            rapid_to_robot2 = list(np.array(rapid_to_robot))
            sensorMessage.RAPIDtoRobot.dnum.extend(rapid_to_robot2)

        buf=sensorMessage.SerializeToString()

        try:
            self.socket.sendto(buf, self.egm_addr)
        except:
            return False

        return True

    def send_to_robot_path_corr(self, pos: np.ndarray, age: float=1):
        """
        Send a path correction command. Returns False if no data has been received from the robot yet. The path
        correction is a displacement [x,y,z] in millimeters in **path coordinates**. The displacement uses
        "path coordinates", which relate the direction of movement of the end effector. See `CorrConn` command in 
        *Technical reference manual RAPID Instructions, Functions and Data types* for a detailed description of path 
        coordinates.  The EGM operation must have been started with EGMActMove, and use EGMMoveL and EGMMoveC commands.

        :param pos: The displacement in path coordinates in millimeters [x,y,z]
        :return: True if successful, False if no data received from robot yet
        """
        
        self.send_sequence_number+=1
        sensorMessage=egm_pb2.EgmSensorPathCorr()

        header=sensorMessage.header
        header.mtype=egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_PATH_CORRECTION')
        header.seqno=self.send_sequence_number
        self.send_sequence_number+=1
        
        pathCorr = sensorMessage.pathCorr

        pathCorr.pos.x = pos[0]
        pathCorr.pos.y = pos[1]
        pathCorr.pos.z = pos[2]
        pathCorr.age=age

        buf=sensorMessage.SerializeToString()

        try:
            self.socket.sendto(buf, self.egm_addr)
        except:
            return False

        return True

    def close(self):
        """
        Close the connection to the robot.
        """
        try:
            self.socket.close()
            self.socket = None
        except:
            pass