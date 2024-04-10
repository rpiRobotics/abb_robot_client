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

import struct
import numpy as np
import io
import requests
import datetime
import re
import time
import websocket
import threading

from typing import Callable, NamedTuple, Any, List, Union, Optional
from enum import IntEnum

class ABBException(Exception):
    """
    Exception returned from ABB controller
    """
    def __init__(self, message, code):
        super(ABBException, self).__init__(message)
        self.code=code

class RAPIDExecutionState(NamedTuple):
    """
    Execution state of RAPID tasks on controller. See :meth:`.RWS.get_execution_state()`
    """
    ctrlexecstate: Any
    """Current controller execution state"""
    cycle: Any
    """Current controller cycle state"""

class EventLogEntry(NamedTuple):
    """Entry within an event log. See :meth:`.RWS.read_event_log()`"""
    seqnum: int
    """Sequence number of the entry"""
    msgtype: int
    """Message type of the entry. 1 for info, 2 for warning, 3 for error"""
    code: int
    """The error code of the entry"""
    tstamp: datetime.datetime 
    """Entry timestamp in controller clock"""
    args: List[Any]
    """Arguments specified when entry created"""
    title: str
    """Entry title"""
    desc: str
    """Entry description"""
    conseqs: str
    """Consequences of the event"""
    causes: str
    """Potential causes of the event"""
    actions: str
    """Actions to correct the event"""

class EventLogEntryEvent(NamedTuple):
    """
    Event returned by subscription when event log entry created
    """
    seqnum: int
    """seqnum of created event log entry"""

class TaskState(NamedTuple):
    """
    Current state of task running on controller. See :meth:`.RWS.get_tasks()`
    """
    name: str
    """Name of the controller task"""
    type_: str
    """The type of the task"""
    taskstate: str
    """The curret task state"""
    excstate: str
    """The current execution state"""
    active: bool
    """True if the task is currently active"""
    motiontask: bool
    """True if the task is a motion task"""

class JointTarget(NamedTuple):
    """Joint target in degrees or millimeters"""
    robax: np.array
    """Robot axes positions. Six entry array"""
    extax: np.array
    """Extra axes positions. Six entry array"""

class RobTarget(NamedTuple):
    """Robtarget"""
    trans: np.array
    """Translation in millimeters. Three entry array"""
    rot: np.array
    """Rotation in quaternion units [w,x,y,z]"""
    robconf: np.array
    """4 entry configuration of robot. See `confdata` datatype in the ABB RAPID manuals for explanation"""
    extax: np.array
    """Extra axes positions. Six entry array"""

class IpcMessage(NamedTuple):
    """IPC queue message. Also used for RMQ. See :meth:`.RMQ.try_create_ipc_queue()`"""
    data: str
    """Message data encoded as string"""
    userdef: str
    """User defined message content encoded as string"""
    msgtype: str
    """Type of messag"""
    cmd: str
    """Message command"""
    queue_name: str
    """Queue containing message"""

class Signal(NamedTuple):
    """Signal state"""
    name: str
    """Name of the signal"""
    lvalue: str
    """Logical value of the signal"""

class ControllerState(NamedTuple):
    """Controller state. See :meth:`.RWS.get_controller_state()`"""
    state: str
    """The controller state"""

class OperationalMode(NamedTuple):
    """Operational mode. See :meth:`.RWS.get_operation_mode()`"""
    mode: str
    """The operational mode"""

class VariableValue(NamedTuple):
    """RAPID variable value"""
    name: str
    """The name of the RAPId variable"""
    value: str
    """The variable value encoded as string"""
    task: str = None
    """The task containing the variable"""

class SubscriptionResourceType(IntEnum):
    """Enum to select resource to subscribe. See :meth:`.RWS.subscribe()`"""
    ControllerState = 1
    """Subscribe to controller state resource"""
    OperationalMode = 2
    """Subsribe to operational mode resource"""
    ExecutionState = 3
    """Subscribe to executio state resource"""
    PersVar = 4
    """Subscribe to RAPID pers variable resource"""
    IpcQueue = 5
    """Subscribe to IPC queue resource"""
    Elog = 6
    """Subscribe to Event Log resource"""
    Signal = 7
    """Subscribe to signal resource"""

class SubscriptionResourcePriority(IntEnum):
    """Priority of subscribed resource. Only Signal and PersVar support high priority. See :meth:`.RWS.subscribe()`"""
    Low = 0
    Medium = 1
    High = 2

class SubscriptionResourceRequest(NamedTuple):
    """Specify resource to subscribe. See :meth:`.RWS.subscribe()`"""
    resource_type: SubscriptionResourceType
    """Type of resource to subscribe"""
    priority: SubscriptionResourcePriority
    """Subscription priority"""
    param: Any = None
    """Parameter for subscription request"""


class RWS:
    """
    Robot Web Services synchronous client. This class uses ABB Robot Web Services HTTP REST interface to interact
    with robot controller. Subscriptions can be created to provide streaming information. See the ABB
    documentation for more information: https://developercenter.robotstudio.com/api/rwsApi/

    :param base_url: Base URL of the robot. For Robot Studio instances, this should be http://127.0.0.1:80,
                     the default value. For a real robot, 127.0.0.1 should be replaced with the IP address
                     of the robot controller. The WAN port ethernet must be used, not the maintenance port.
    :param username: The HTTP username for the robot. Defaults to 'Default User'
    :param password: The HTTP password for the robot. Defaults to 'robotics'
    """
    def __init__(self, base_url: str='http://127.0.0.1:80', username: str=None, password: str=None):
        self.base_url=base_url
        if username is None:
            username = 'Default User'
        if password is None:
            password = 'robotics'
        self.auth=requests.auth.HTTPDigestAuth(username, password)
        self._session=requests.Session()
        self._rmmp_session=None
        self._rmmp_session_t=None
        
    def _do_get(self, relative_url):
        url="/".join([self.base_url, relative_url])
        if "?" in url:
            url += "&json=1"
        else:
            url += "?json=1"
        res=self._session.get(url, auth=self.auth)
        try:            
            return self._process_response(res)
        finally:
            res.close()
    

    def _do_post(self, relative_url, payload=None):
        url="/".join([self.base_url, relative_url])
        if "?" in url:
            url += "&json=1"
        else:
            url += "?json=1"
        res=self._session.post(url, data=payload, auth=self.auth)
        try:
            return self._process_response(res)
        finally:
            res.close()

    def _process_response(self, response):        
        
        if (response.status_code == 503):
            raise Exception("Robot returning 500 Internal Server Error")

        if (response.status_code == 503):
            raise Exception("Robot returning 503 too many active connections")

        if response.status_code == 204:
            return None

        response_json = None
        if response.headers["Content-Type"] == 'application/json' and len(response.content) > 0:
            try:            
                response_json = response.json()
            except:
                if not response.text.startswith("<?xml"):
                    raise
    
        if (response.status_code == 200 or response.status_code == 201  \
            or response.status_code==202 or response.status_code==204):
            
            return response_json
        
        if response_json is None:
            raise Exception("Robot returning HTTP error " + str(response.status_code))        
        
        status = response_json["_embedded"]["status"]
        error_code=int(status["code"])
        error_message=status.get('msg',"Received error from ABB robot: " + str(error_code))
        
        raise ABBException(error_message, error_code)

    def start(self, cycle: Optional[str]='asis',tasks: Optional[List[str]]=['T_ROB1']):
        """
        Start one or more RAPID tasks

        :param cycle: The cycle mode of the robot. Can be `asis`, `once`, or `forever`.
        :param tasks: One or more tasks to start.
        """

        rob_tasks = self.get_tasks()
        for t in tasks:
            if not t in rob_tasks:
                raise Exception(f"Cannot start unknown task {t}")

        for rob_task in rob_tasks.values():
            if not rob_task.motiontask:
                continue
            if rob_task.name in tasks:
                if not rob_task.active:
                    self.activate_task(rob_task.name)
            else:
                if rob_task.active:
                    self.deactivate_task(rob_task.name)

        payload={"regain": "continue", "execmode": "continue" , "cycle": cycle, "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "true"}
        res=self._do_post("rw/rapid/execution?action=start", payload)

    def activate_task(self, task: str):
        """
        Activate a RAPID task

        :param task: The name of the task to activate
        """
        payload={}
        self._do_post(f"rw/rapid/tasks/{task}?action=activate",payload)

    def deactivate_task(self, task: str) -> None:
        """
        Deactivate a RAPID task

        :param task: The name of the task to activate
        """
        payload={}
        self._do_post(f"rw/rapid/tasks/{task}?action=deactivate",payload)

    def stop(self):
        """
        Stop RAPID execution of normal tasks
        """
        payload={"stopmode": "stop"}
        res=self._do_post("rw/rapid/execution?action=stop", payload)

    def resetpp(self):
        """
        Reset RAPID program pointer to main in normal tasks
        """
        res=self._do_post("rw/rapid/execution?action=resetpp")

    def get_ramdisk_path(self) -> str:
        """
        Get the path of the RAMDISK variable on the controller

        :return: The RAMDISK path
        """
        res_json = self._do_get("ctrl/$RAMDISK")
        return res_json["_embedded"]["_state"][0]["_value"]

    def get_execution_state(self) -> RAPIDExecutionState:
        """
        Get the RAPID execution state

        :return: The RAPID execution state
        """
        res_json = self._do_get("rw/rapid/execution")
        state = res_json["_embedded"]["_state"][0]
        ctrlexecstate=state["ctrlexecstate"]
        cycle=state["cycle"]
        return RAPIDExecutionState(ctrlexecstate, cycle)
    
    def get_controller_state(self) -> str:
        """
        Get the controller state. The controller state can have the following values:

        `init`, `motoroff`, `motoron`, `guardstop`, `emergencystop`, `emergencystopreset`, `sysfail`

        RAPID can only be executed and the robot can only be moved in the `motoron` state.

        :return: The controller state
        """
        res_json = self._do_get("rw/panel/ctrlstate")
        state = res_json["_embedded"]["_state"][0]
        return state['ctrlstate']

    def set_controller_state(self, ctrl_state):
        payload = {"ctrl-state": ctrl_state}
        res=self._do_post("rw/panel/ctrlstate?action=setctrlstate", payload)
    
    def get_operation_mode(self) -> str:
        """
        Get the controller operational mode. The controller operational mode can have the following values:

        `INIT`, `AUTO_CH`, `MANF_CH`, `MANR`, `MANF`, `AUTO`, `UNDEF`

        Typical values returned by the controller are `AUTO` for auto mode, and `MANR` for manual reduced-speed mode.
        
        :return: The controller operational mode.
        """
        res_json = self._do_get("rw/panel/opmode")        
        state = res_json["_embedded"]["_state"][0]
        return state["opmode"]
    
    def get_digital_io(self, signal: str, network: str='Local', unit: str='DRV_1') -> int:
        """
        Get the value of a digital IO signal.

        :param signal: The name of the signal
        :param network: The network the signal is on. The default `Local` will work for most signals.
        :param unit: The drive unit of the signal. The default `DRV_1` will work for most signals.
        :return: The value of the signal. Typically 1 for ON and 0 for OFF
        """
        res_json = self._do_get("rw/iosystem/signals/" + network + "/" + unit + "/" + signal)
        state = res_json["_embedded"]["_state"][0]["lvalue"]
        return int(state)
    
    def set_digital_io(self, signal: str, value: Union[bool,int], network: str='Local', unit: str='DRV_1'):
        """
        Set the value of an digital IO signal.

        :param value: The value of the signal. Bool or bool convertible input
        :param signal: The name of the signal
        :param network: The network the signal is on. The default `Local` will work for most signals.
        :param unit: The drive unit of the signal. The default `DRV_1` will work for most signals.
        """
        lvalue = '1' if bool(value) else '0'
        payload={'lvalue': lvalue}
        res=self._do_post("rw/iosystem/signals/" + network + "/" + unit + "/" + signal + "?action=set", payload)

    def get_analog_io(self, signal: str, network: str='Local', unit: str='DRV_1') -> float:
        """
        Get the value of an analog IO signal.

        :param signal: The name of the signal
        :param network: The network the signal is on. The default `Local` will work for most signals.
        :param unit: The drive unit of the signal. The default `DRV_1` will work for most signals.
        :return: The value of the signal
        """
        res_json = self._do_get("rw/iosystem/signals/" + network + "/" + unit + "/" + signal)
        state = res_json["_embedded"]["_state"][0]["lvalue"]
        return float(state)
    
    def set_analog_io(self, signal: str, value: Union[int,float], network: str='Local', unit: str='DRV_1'):
        """
        Set the value of an analog IO signal.

        :param value: The value of the signal
        :param signal: The name of the signal
        :param network: The network the signal is on. The default `Local` will work for most signals.
        :param unit: The drive unit of the signal. The default `DRV_1` will work for most signals.
        """
        payload={"mode": "value",'lvalue': value}
        res=self._do_post("rw/iosystem/signals/" + network + "/" + unit + "/" + signal + "?action=set", payload)
    
    def get_rapid_variables(self, task: str="T_ROB1") -> List[str]:
        """
        Get a list of the persistent variables in a task

        :param task: The RAPID task to query
        :return: List of persistent variables in task
        """
        payload={
            "view": "block",
            "vartyp": "any",
            "blockurl": f"RAPID/{task}" if task is not None else "RAPID",
            "symtyp": "per",
            "recursive": "true",
            "skipshared": "FALSE",
            "onlyused": "FALSE",
            "stack": "0",
            "posl": "0",
            "posc": "0"
        }
        res_json = self._do_post(f"rw/rapid/symbols?action=search-symbols", payload)
        state = res_json["_embedded"]["_state"]
        return state

    def get_rapid_variable(self, var: str, task: str = "T_ROB1") -> str:
        """
        Get value of a RAPID pers variable

        :param var: The pers variable name
        :param task: The task containing the pers variable
        :return: The pers variable encoded as a string
        """
        if task is not None:
            var1 = f"{task}/{var}"
        else:
            var1 = var
        res_json = self._do_get("rw/rapid/symbol/data/RAPID/" + var1)
        state = res_json["_embedded"]["_state"][0]["value"]
        return state
    
    def set_rapid_variable(self, var: str, value: str, task: str = "T_ROB1"):
        """
        Set value of a RAPID pers variable

        :param var: The pers variable name
        :param value: The new variable value encoded as a string
        :param task: The task containing the pers variable
        """
        payload={'value': value}
        if task is not None:
            var1 = f"{task}/{var}"
        else:
            var1 = var
        res=self._do_post("rw/rapid/symbol/data/RAPID/" + var1 + "?action=set", payload)
        
    def read_file(self, filename: str) -> bytes:
        """
        Read a file off the controller

        :param filename: The filename to read
        :return: The file bytes
        """
        url="/".join([self.base_url, "fileservice", filename])
        res=self._session.get(url, auth=self.auth)
        if not res.ok:
            raise Exception(f"File not found {filename}")
        try:            
            return res.content
        finally:
            res.close()

    def upload_file(self, filename: str, contents: bytes):
        """
        Upload a file to the controller

        :param filename: The filename to write
        :param contents: The file content bytes
        """
        url="/".join([self.base_url, "fileservice" , filename])
        res=self._session.put(url, contents, auth=self.auth)
        if not res.ok:
            raise Exception(res.reason)
        res.close()

    def delete_file(self, filename: str):
        """
        Delete a file on the controller

        :param filename: The filename to delete
        """
        url="/".join([self.base_url, "fileservice" , filename])
        res=self._session.delete(url, auth=self.auth)
        res.close()

    def list_files(self, path: str) -> List[str]:
        """
        List files at a path on a controller

        :param path: The path to list
        :return: The filenames in the path
        """
        res_json = self._do_get("fileservice/" + str(path) + "")
        state = res_json["_embedded"]["_state"]
        return [f["_title"] for f in state]

    def read_event_log(self, elog: int=0) -> List[EventLogEntry]:
        """
        Read the controller event log

        :param elog: The event log id to read
        :return: The event log entries        
        """
        o=[]
        res_json = self._do_get("rw/elog/" + str(elog) + "/?lang=en")
        state = res_json["_embedded"]["_state"]
        
        for s in state:
            seqnum = int(s["_title"].split("/")[-1])
            msg_type=int(s["msgtype"])
            code=int(s["code"])
            tstamp=datetime.datetime.strptime(s["tstamp"], '%Y-%m-%d T  %H:%M:%S')
            title=s["title"]
            desc=s["desc"]
            conseqs=s["conseqs"]
            causes=s["causes"]
            actions=s["actions"]
            args=[]
            nargs=int(s["argc"])
            if "argv" in s:
                for arg in s["argv"]:
                    args.append(arg["value"])
            
            o.append(EventLogEntry(seqnum,msg_type,code,tstamp,args,title,desc,conseqs,causes,actions))
        return o

    def get_tasks(self) -> List[TaskState]:
        """
        Get controller tasks and task state

        :return: The tasks and task state
        """
        o = {}
        res_json = self._do_get("rw/rapid/tasks")
        state = res_json["_embedded"]["_state"]
                
        for s in state:
            name=s["name"]
            type_=s["type"]
            taskstate=s["taskstate"]
            excstate=s["excstate"]            
            try:
                active=s["active"] == "On"
            except:
                active=False
            try:
              motiontask=s["motiontask"].lower() == "true"
            except:
                motiontask=False

            o[name]=TaskState(name,type_,taskstate,excstate,active,motiontask)
        
        return o

    def get_jointtarget(self, mechunit="ROB_1") -> JointTarget:
        """
        Get the current jointtarget for specified mechunit

        :param mechunit: The mechanical unit to read
        :return: The current jointtarget
        """
        res_json=self._do_get("rw/motionsystem/mechunits/" + mechunit + "/jointtarget")
        state = res_json["_embedded"]["_state"][0]
        if not state["_type"] == "ms-jointtarget":
            raise Exception("Invalid jointtarget type")
        robjoint=np.array([state["rax_1"], state["rax_2"], state["rax_3"], state["rax_4"], state["rax_5"], 
            state["rax_6"]], dtype=np.float64)
        extjoint=np.array([state["eax_a"], state["eax_b"], state["eax_c"], state["eax_d"], state["eax_e"], 
            state["eax_f"]], dtype=np.float64)
     
        return JointTarget(robjoint,extjoint)
        
    def get_robtarget(self, mechunit='ROB_1', tool='tool0', wobj='wobj0', coordinate='Base') -> RobTarget:
        """
        Get the current robtarget (cartesian pose) for the specified mechunit

        :param mechunit: The mechanical unit to read
        :param tool: The tool to use to compute robtarget
        :param wobj: The wobj to use to compute robtarget
        :param coordinate: The coordinate system to use to compute robtarget. Can be `Base`, `World`, `Tool`, or `Wobj`
        :return: The current robtarget

        """
        res_json=self._do_get(f"rw/motionsystem/mechunits/{mechunit}/robtarget?tool={tool}&wobj={wobj}&coordinate={coordinate}")
        state = res_json["_embedded"]["_state"][0]
        if not state["_type"] == "ms-robtargets":
            raise Exception("Invalid robtarget type")
        trans=np.array([state["x"], state["y"], state["z"]], dtype=np.float64)
        rot=np.array([state["q1"], state["q2"], state["q3"], state["q4"]], dtype=np.float64)
        robconf=np.array([state["cf1"],state["cf4"],state["cf6"],state["cfx"]], dtype=np.float64)
        extax=np.array([state["eaxa"], state["eaxb"], state["eaxc"], state["eaxd"], state["eaxe"], 
            state["eaxf"]], dtype=np.float64)
        return RobTarget(trans,rot,robconf,extax)
    
    def _rws_value_to_jointtarget(self, val):
        v1=re.match('^\\[\\[([^\\]]+)\\],\\[([^\\]]+)\\]',val)
        robax = np.deg2rad(np.fromstring(v1.groups()[0],sep=','))
        extax = np.deg2rad(np.fromstring(v1.groups()[1],sep=','))
        return JointTarget(robax,extax)
    
    def _jointtarget_to_rws_value(self, val):
        if not np.shape(val[0]) == (6,):
            raise Exception("Invalid jointtarget")
        if not np.shape(val[1]) == (6,):
            raise Exception("Invalid jointtarget")
        robax=','.join([format(x, '.4f') for x in np.rad2deg(val[0])])
        extax=','.join([format(x, '.4f') for x in np.rad2deg(val[1])])
        rws_value="[[" + robax + "],[" + extax + "]]"
        return rws_value
    
    def get_rapid_variable_jointtarget(self, var, task: str = "T_ROB1") -> JointTarget:
        """
        Get a RAPID pers variable and convert to JointTarget

        :param var: The pers variable name
        :param task: The task containing the pers variable
        :return: The pers variable encoded as a JointTarget
        """
        v = self.get_rapid_variable(var, task)
        return self._rws_value_to_jointtarget(v)
    
    def set_rapid_variable_jointtarget(self,var: str, value: JointTarget, task: str = "T_ROB1"):
        """
        Set a RAPID pers variable from a JointTarget

        :param var: The pers variable name
        :param value: The new variable JointTarget value
        :param task: The task containing the pers variable
        """
        rws_value=self._jointtarget_to_rws_value(value)
        self.set_rapid_variable(var, rws_value, task)
            
    def _rws_value_to_jointtarget_array(self,val):
        m1=re.match('^\\[(.*)\\]$',val)
        if len(m1.groups()[0])==0:
            return []
        arr=[]
        val1=m1.groups()[0]
        while len(val1) > 0:
            m2=re.match('^(\\[\\[[^\\]]+\\],\\[[^\\]]+\\]\\]),?(.*)$',val1)            
            val1 = m2.groups()[1]
            arr.append(self._rws_value_to_jointtarget(m2.groups()[0]))
        
        return arr       
    
    def _jointtarget_array_to_rws_value(self, val):
        return "[" + ','.join([self._jointtarget_to_rws_value(v) for v in val]) + "]"
    
    def get_rapid_variable_jointtarget_array(self, var: str, task: str = "T_ROB1") -> List[JointTarget]:
        """
        Get a RAPID pers variable and convert to JointTarget list

        :param var: The pers variable name
        :param task: The task containing the pers variable
        :return: The pers variable encoded as a list of JointTarget
        """
        v = self.get_rapid_variable(var, task)
        return self._rws_value_to_jointtarget_array(v)
    
    def set_rapid_variable_jointtarget_array(self,var: str,value: List[JointTarget], task: str = "T_ROB1"):
        """
        Set a RAPID pers variable from a JointTarget list

        :param var: The pers variable name
        :param value: The new variable JointTarget value
        :param task: The task containing the pers variable
        """
        rws_value=self._jointtarget_array_to_rws_value(value)
        self.set_rapid_variable(var, rws_value, task)

    def get_rapid_variable_num(self, var: str, task: str = "T_ROB1") -> float:
        """
        Get a RAPID pers variable and convert to float

        :param var: The pers variable name
        :param task: The task containing the pers variable
        :return: The pers variable encoded as a float
        """
        return float(self.get_rapid_variable(var,task))
    
    def set_rapid_variable_num(self, var: str, val: float, task: str = "T_ROB1"):
        """
        Set a RAPID pers variable from a float

        :param var: The pers variable name
        :param value: The new variable float value
        :param task: The task containing the pers variable
        """
        self.set_rapid_variable(var, str(val), task)
        
    def get_rapid_variable_num_array(self, var, task: str = "T_ROB1") -> np.ndarray:
        """
        Get a RAPID pers variable float array

        :param var: The pers variable name
        :param task: The task containing the pers variable
        :return: The variable value as an array
        """
        val1=self.get_rapid_variable(var,task)
        m=re.match("^\\[([^\\]]*)\\]$", val1)
        val2=m.groups()[0].strip()
        return np.fromstring(val2,sep=',')
    
    def set_rapid_variable_num_array(self, var: str, val: List[float], task: str = "T_ROB1"):
        """
        Set a RAPID pers variable from a float list or array

        :param var: The pers variable name
        :param value: The new variable float array value
        :param task: The task containing the pers variable
        """
        self.set_rapid_variable(var, "[" + ','.join([str(s) for s in val]) + "]", task)

    def read_ipc_message(self, queue_name: str, timeout: float=0) -> List[IpcMessage]:
        """
        Read IPC message. IPC is used to communicate with RMQ in controller tasks. Create IPC using 
        try_create_ipc_queue().

        :param queue_name: The name of the queue created using try_create_ipc_queue()
        :param timeout: The timeout to receive a message in seconds
        :return: Messages received from IPC queue
        """
        o=[]
        
        timeout_str=""
        if timeout > 0:
            timeout_str="&timeout=" + str(timeout)
        
        res_json=self._do_get("rw/dipc/" + queue_name + "/?action=dipc-read" + timeout_str)
        for state in res_json["_embedded"]["_state"]:
            if not state["_type"] == "dipc-read-li":
                raise Exception("Invalid IPC message type")
     
            o.append(IpcMessage(state["dipc-data"], state["dipc-userdef"],
                state["dipc-msgtype"], state["dipc-cmd"], state["queue-name"]))
            
            #o.append(RAPIDEventLogEntry(msg_type,code,tstamp,args,title,desc,conseqs,causes,actions))
        return o
    
    def get_speedratio(self) -> float:
        """
        Get the current speed ratio

        :return: The current speed ratio between 0% - 100%
        """
        res_json=self._do_get(f"rw/panel/speedratio")
        state = res_json["_embedded"]["_state"][0]
        if not state["_type"] == "pnl-speedratio":
            raise Exception("Invalid speedratio type")
        return float(state["speedratio"])
    
    def set_speedratio(self, speedratio: float):
        """
        Set the current speed ratio

        :param speedratio: The new speed ratio between 0% - 100%
        """
        payload = {"speed-ratio": str(speedratio)}
        self._do_post(f"rw/panel/speedratio?action=setspeedratio", payload)
        
    
    def send_ipc_message(self, target_queue: str, data: str, queue_name: str, cmd: int=111, userdef: int=1, msgtype: int=1 ):
        """
        Send an IPC message to the specified queue

        :param target_queue: The target IPC queue. Can also be the name of a task to send to RMQ of controller task.
        :param data: The data to send to the controller. Encoding must match the expected type of RMQ
        :param queue_name: The queue to send message from. Must be created with try_create_ipc_queue()
        :param cmd: The cmd entry in the message
        :param userdef: User defined value
        :param msgtype: The type of message. Must be 0 or 1
        """
        payload={"dipc-src-queue-name": queue_name, "dipc-cmd": str(cmd), "dipc-userdef": str(userdef), \
                 "dipc-msgtype": str(msgtype), "dipc-data": data}
        res=self._do_post("rw/dipc/" + target_queue + "?action=dipc-send", payload)
    
    def get_ipc_queue(self, queue_name: str) -> Any:
        """
        Get the IPC queue

        :param queue_name: The name of the queue
        """
        res=self._do_get("rw/dipc/" + queue_name + "?action=dipc-read")
        return res
    
    def try_create_ipc_queue(self, queue_name: str, queue_size: int=4440, max_msg_size: int=444) -> bool:
        """
        Try creating an IPC queue. Returns True if the queue is created, False if queue already exists. Raises
        exception for all other errors.

        :param queue_name: The name of the new IPC queue
        :param queue_size: The buffer size of the queue
        :param max_msg_size: The maximum message size of the queue
        :return: True if queue created, False if queue already exists
        
        """
        try:
            payload={"dipc-queue-name": queue_name, "dipc-queue-size": str(queue_size), "dipc-max-msg-size": str(max_msg_size)}
            self._do_post("rw/dipc?action=dipc-create", payload)
            return True
        except ABBException as e:
            if e.code==-1073445879:
                return False
            raise
    
    def request_rmmp(self, timeout: float=5):
        """
        Request Remote Mastering. Required to alter pers variables in manual control mode. The teach pendant
        will prompt to enable remote mastering, and the user must confirm. Once remote mastering is enabled,
        poll_rmmp() must be executed periodically to maintain rmmp.

        :param timeout: The request timeout in seconds
        """
        t1=time.time()
        self._do_post('users/rmmp?json=1', {'privilege': 'modify'})
        while time.time() - t1 < timeout:
            
            res_json=self._do_get('users/rmmp/poll?json=1')
            state = res_json["_embedded"]["_state"][0]
            if not state["_type"] == "user-rmmp-poll":
                raise Exception("Invalid rmmp poll type")
            status = state["status"]
            if status=="GRANTED":
                self.poll_rmmp()
                return
            elif status!="PENDING":
                raise Exception("User did not grant remote access")                               
            time.sleep(0.25)
        raise Exception("User did not grant remote access")
    
    def poll_rmmp(self):
        """
        Poll rmmp to maintain remote mastering. Call periodically after rmmp enabled using request_rmmp()
        """
        
        # A "persistent session" can only make 400 calls before
        # being disconnected. Once this connection is lost,
        # the grant will be revoked. To work around this,
        # create parallel sessions with copied session cookies
        # to maintain the connection.
        
        url="/".join([self.base_url, 'users/rmmp/poll?json=1'])
        
        old_rmmp_session=None
        if self._rmmp_session is None:
            self._do_get(url)
            self._rmmp_session=requests.Session()
            self._rmmp_session_t=time.time()            
            
            for c in self._session.cookies:
                self._rmmp_session.cookies.set_cookie(c) 
        else:
            if time.time() - self._rmmp_session_t > 30:
                old_rmmp_session=self._rmmp_session
                rmmp_session=requests.Session()
                
                for c in self._session.cookies:
                    rmmp_session.cookies.set_cookie(c)
        
        rmmp_session=self._rmmp_session        
                
        res=rmmp_session.get(url, auth=self.auth)
        res_json=self._process_response(res)
        state = res_json["_embedded"]["_state"][0]
        if not state["_type"] == "user-rmmp-poll":
            raise Exception("Invalid rmmp poll type")
                
        if old_rmmp_session is not None:
            self._rmmp_session=rmmp_session
            self._rmmp_session_t=time.time()
            try:
                old_rmmp_session.close()
            except:
                pass
       
        return state["status"] == "GRANTED"


    def subscribe(self, resources: List[SubscriptionResourceRequest], handler: Callable[[Any],None]) -> "RWSSubscription":
        """
        Create subscription that will receive real-time updates from the controller. handler will be called
        with the new values. RWS subscriptions are relatively slow, with delays in the hundreds of milliseconds. 
        Use EGM for faster real-time updates.

        A list of `SubscriptionResourceRequest` are used to specify what resources to subscribe. The following are
        supported:

        * Controller State: `SubscriptionResourceRequest(SubscriptionResourceType.ControllerState, rws.SubscriptionResourcePriority.Medium)`
        * Operational Mode: `SubscriptionResourceRequest(SubscriptionResourceType.OperationalMode, rws.SubscriptionResourcePriority.Medium)`
        * Execution State: `SubscriptionResourceRequest(SubscriptionResourceType.ExecutionState, rws.SubscriptionResourcePriority.Medium)`
        * Variable: `SubscriptionResourceRequest(SubscriptionResourceType.PersVar, rws.SubscriptionResourcePriority.High, "variable_name")`
        * IPC Queue: SubscriptionResourceRequest(SubscriptionResourceType.IpcQueue, rws.SubscriptionResourcePriority.Medium, "queue_name")`
        * Event Log: SubscriptionResourceRequest(SubscriptionResourceType.Elog, rws.SubscriptionResourcePriority.Medium)
        * Signal: SubscriptionResourceRequest(SubscriptionResourceType.Signal, rws.SubscriptionResourcePriority.High, "signal_name")

        The `handler` parameter will be called on each event from the controller. The passed parameter will
        have the following type:

        * Controller State: :class:`.ControllerState`
        * Operational Mode: :class:`.OperationalMode`
        * Execution State: :class:`.RAPIDExecutionState`
        * Variable: :class:`.PersVar`
        * IPC Queue: :class:`.IpcMessage`
        * Event Log: :class:`.EventLogEntry`
        * Signal: :class:`.Signal`
        
        The returned `RWSSubscription` creates a persistent websocket connection to the controller. It must
        be closed with the close() method when no longer in use.

        :param resources: Controller resources to subscribe
        :param handler: Callable to call on resource events
        :return: Active subscription
        """
        payload = {}
        payload_ind = 0
        for r in resources:
            payload_ind += 1
            if r.resource_type == SubscriptionResourceType.ControllerState:
                payload[f"{payload_ind}"] = "/rw/panel/ctrlstate"
            elif r.resource_type == SubscriptionResourceType.OperationalMode:
                payload[f"{payload_ind}"] = "/rw/panel/opmode"
            elif r.resource_type == SubscriptionResourceType.ExecutionState:
                payload[f"{payload_ind}"] = "/rw/rapid/execution;ctrlexecstate"
            elif r.resource_type == SubscriptionResourceType.PersVar:
                if isinstance(r.param, str):
                    var1 = f"T_ROB1/{r.param}"
                else:
                    var_name = r.param["name"]
                    task = r.param.get("task", "T_ROB1")
                    if task is not None:
                        var1 = f"{task}/{var_name}"
                    else:
                        var1 = var_name
                payload[f"{payload_ind}"] = f"/rw/rapid/symbol/data/RAPID/{var1};value"
            elif r.resource_type == SubscriptionResourceType.IpcQueue:
                payload[f"{payload_ind}"] = f'/rw/dipc/{r.param}'
            elif r.resource_type == SubscriptionResourceType.Elog:
                payload[f"{payload_ind}"] = f'/rw/elog/0'
            elif r.resource_type == SubscriptionResourceType.Signal:
                if isinstance(r.param, str):
                    signal = r.param
                    network = "Local"
                    unit = "DRV_1"
                else:
                    signal = r.param["signal"]
                    network = r.param.get("network", "Local")
                    unit = r.param.get("unit", "DRV_1")
                payload[f"{payload_ind}"] = f'/rw/iosystem/signals/{network}/{unit}/{signal};state'
            else:
                raise Exception("Invalid resource type")
            payload[f"{payload_ind}-p"] = f"{r.priority.value}"

        payload["resources"] = [f"{i+1}" for i in range(payload_ind)]

                
        url="/".join([self.base_url, "subscription"]) + "?json=1"
        res1=self._session.post(url, data=payload, auth=self.auth)
        try:
            res=self._process_response(res1)
        finally:
            res1.close()

        if not res1.status_code == 201:
            raise Exception("Subscription creation failed")

        m = re.search(r'<a href="ws:\/\/.*(\/poll\/\d+)" rel="self">', res1.content.decode("ascii"))
        if not m:
            raise Exception("Invalid subscription response")
        ws_url = self.base_url.replace("http:","ws:") + m.group(1)
        
        cookie = f"ABBCX={self._session.cookies['ABBCX']}"
        header={'Cookie': cookie, 'Authorization': self.auth.build_digest_header("GET", ws_url)}

        return RWSSubscription(ws_url, header, handler)

    def logout(self):
        res=self._do_get("logout")

    def close(self):
        try:
            self.logout()
        except:
            pass

        try:
            self._session.close()
        except:
            pass

class SubscriptionException(Exception):
    def __init__(self, message):
        super().__init__(message)

class SubscriptionClosed(NamedTuple):
    code: int
    msg: str

class RWSSubscription:
    """
    Subscription returned from :meth:`RWS.subscribe()`
    """
    def __init__(self, ws_url, header, handler):
        self.handler = handler

        self._signal_re = re.compile(r'<a\s+href="/rw/iosystem/signals/([^"]+);state"\s+rel="self"/?>.*<span\s+class="lvalue">([^<]+)<')
        self._pers_re = re.compile(r'<a\s+href="/rw/rapid/symbol/data/RAPID/([^"]+);value"\s+rel="self"/?>.*<span\s+class="value">([^<]+)<')
        self._elog_re = re.compile(r'<a\s+href="/rw/elog/0/([^"]+)"\s+rel="self"/?>.*<span\s+class="seqnum">([^<]+)<')
        self._exec_re = re.compile(r'<a\s+href="/rw/rapid/execution;ctrlexecstate"\s+rel="self"/?>.*<span\s+class="ctrlexecstate">([^<]+)<')
        self._opmode_re = re.compile(r'<a\s+href="/rw/panel/opmode"\s+rel="self"/?>.*<span\s+class="opmode">([^<]+)<')
        self._ctrl_re = re.compile(r'<a\s+href="/rw/panel/ctrlstate"\s+rel="self"/?>.*<span\s+class="ctrlstate">([^<]+)<')
        self._ipc_re = re.compile(r'<a\s+href="/rw/dipc/([^"]*)".*<span\s+class="dipc-data">([^<]+)<.*<span\s+class="dipc-userdef">([^<]+)<')

        self.ws = websocket.WebSocketApp(
            ws_url, 
            header = header,
            on_open = self._on_open,
            on_message = self._on_message,
            on_error = self._on_error,
            on_close = self._on_close,
            subprotocols=['robapi2_subscription']
            )

        self.thread = threading.Thread(target=self._run)
        self.thread.daemon=True
        self.thread.start()

    def _run(self):
        self.ws.run_forever(reconnect = 0.1)

    def _on_message(self, ws, message):
        if 'li class="ios-signalstate-ev"' in message:
            m = self._signal_re.search(message)
            if m is not None:
                sig_path = m.group(1).split('/')
                self.handler(Signal(sig_path[-1], m.group(2)))
        elif 'li class="rap-data"' in message:
            m = self._pers_re.search(message)
            if m is not None:
                pers_path = m.group(1).split('/')
                if len(pers_path) == 1:            
                    self.handler(VariableValue(pers_path[0], m.group(2)))
                else:
                    self.handler(VariableValue(pers_path[0], m.group(2), pers_path[-1]))
        elif 'li class="elog-message-ev"' in message:
            m = self._elog_re.search(message)
            if m is not None:
                self.handler(EventLogEntryEvent(m.group(2)))
        elif 'li class="rap-ctrlexecstate-ev"' in message:
            m = self._exec_re.search(message)
            if m is not None:
                self.handler(RAPIDExecutionState(m.group(1),''))
        elif 'li class="pnl-opmode-ev"' in message:
            m = self._opmode_re.search(message)
            if m is not None:
                self.handler(OperationalMode(m.group(1)))
        elif 'li class="pnl-ctrlstate-ev"' in message:
            m = self._ctrl_re.search(message)
            if m is not None:
                self.handler(ControllerState(m.group(1)))
        elif 'li class="dipc-msg-ev"' in message:
            m = self._ipc_re.search(message)
            if m is not None:
                self.handler(IpcMessage(queue_name=m.group(1), data=m.group(2), userdef=m.group(3), msgtype="", cmd=""))
                

    def _on_error(self, ws, error):
        self.handler(SubscriptionException(error))

    def _on_close(self, ws, close_status_code, close_msg):
        self.handler(SubscriptionClosed(close_status_code, close_msg))

    def _on_open(self, ws):
        pass

    def close(self):
        """
        Close the subscription
        """
        self.ws.close()
