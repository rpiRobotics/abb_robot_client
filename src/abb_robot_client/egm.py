import socket
import select
from ._egm_protobuf import egm_pb2
import traceback
import numpy as np
import errno
from typing import Tuple, NamedTuple, Any

class EGMRobotState(NamedTuple):
    joint_angles: np.array
    rapid_running: bool
    motors_on: bool 
    robot_message: Any
    cartesian: Tuple[np.array,np.array]



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
        cartesian=None

        if robot_message.HasField('feedBack'):
            joints=robot_message.feedBack.joints.joints
            joint_angles=np.array(list(joints))
            if robot_message.feedBack.HasField('cartesian'):
                cart_p = robot_message.feedBack.cartesian.pos
                cart_q = robot_message.feedBack.cartesian.orient
                cartesian = (np.array([cart_p.x,cart_p.y,cart_p.z]),np.array([cart_q.u0,cart_q.u1,cart_q.u2,cart_q.u3]))
        if robot_message.HasField('rapidExecState'):
            rapid_running = robot_message.rapidExecState.state == robot_message.rapidExecState.RAPID_RUNNING
        if robot_message.HasField('motorState'):
            motors_on = robot_message.motorState.state == robot_message.motorState.MOTORS_ON
        
            

        return True, EGMRobotState(joint_angles, rapid_running, motors_on, robot_message, cartesian)

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
            joint_angles2 = list(np.array(joint_angles))
            planned.joints.joints.extend(joint_angles2)

        buf2=sensorMessage.SerializeToString()

        try:
            self.socket.sendto(buf2, self.egm_addr)
        except:
            return False

        return True

    def send_to_robot_cart(self, pos, orient):
        if not self.egm_addr:
            return False

        self.send_sequence_number+=1

        sensorMessage=egm_pb2.EgmSensor()

        header=sensorMessage.header
        header.mtype=egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_CORRECTION')
        header.seqno=self.send_sequence_number
        self.send_sequence_number+=1

        planned=sensorMessage.planned

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

        buf=sensorMessage.SerializeToString()

        try:
            self.socket.sendto(buf, self.egm_addr)
        except:
            return False

        return True

    def send_to_robot_path_corr(self, pos, age=1):
    
        
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
