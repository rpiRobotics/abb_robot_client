import socket
import select
from ._egm_protobuf import egm_pb2
import traceback
import numpy as np
import errno
from typing import Tuple, NamedTuple

class EGMRobotState(NamedTuple):
    joint_angles: np.array
    rapid_running: bool
    motors_on: bool 
    robot_message: str

class JointTarget(NamedTuple):
    robax: np.array 
    extax: np.array

class RobTarget(NamedTuple):
    trans: np.array
    rot: np.array
    robconf: np.array
    extax: np.array

class EGM(object):

    def __init__(self, port : int=6510):

        self.socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('',port))
        self.send_sequence_number=0
        self.egm_addr=None
        self.count=0

    def receive_from_robot(self, timeout : float=0) -> Tuple[bool,EGMRobotState]:

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

        if robot_message.HasField('feedBack'):
            joints=robot_message.feedBack.joints.joints
            joint_angles=np.array(list(joints))
        if robot_message.HasField('rapidExecState'):
            rapid_running = robot_message.rapidExecState.state == robot_message.rapidExecState.RAPID_RUNNING
        if robot_message.HasField('motorState'):
            motors_on = robot_message.motorState.state == robot_message.motorState.MOTORS_ON

        return True, EGMRobotState(joint_angles, rapid_running, motors_on, robot_message)

    def send_to_robot(self, joint_angles: np.array) -> bool:

        if not self.egm_addr:
            return False

        self.send_sequence_number+=1

        sensorMessage=egm_pb2.EgmSensor()

        header=sensorMessage.header
        header.mtype=egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_CORRECTION')
        header.seqno=self.send_sequence_number
        self.send_sequence_number+=1

        planned=sensorMessage.planned

        if joint_angles is not None:
            joint_angles2 = list(np.rad2deg(joint_angles))
            planned.joints.joints.extend(joint_angles2)

        buf2=sensorMessage.SerializeToString()

        try:
            self.socket.sendto(buf2, self.egm_addr)
        except:
            return False

        return True
