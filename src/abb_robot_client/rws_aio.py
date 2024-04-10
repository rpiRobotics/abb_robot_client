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
import datetime
import re
import time
import threading
import asyncio

from typing import Callable, NamedTuple, Any, List, Union
import httpx
import websockets

from .rws import ABBException, RAPIDExecutionState, EventLogEntry, EventLogEntryEvent, TaskState, JointTarget, \
    RobTarget, IpcMessage, Signal, ControllerState, OperationalMode, VariableValue, SubscriptionResourceType, \
    SubscriptionResourcePriority, SubscriptionResourceRequest, SubscriptionException, SubscriptionClosed

class RWS_AIO:
    """
    Robot Web Services asyncio client. This class has the same functionality as :class:`abb_robot_client.rws.RWS`, but 
    uses asyncio instead of synchronous interface.

    :param base_url: Base URL of the robot. For Robot Studio instances, this should be http://127.0.0.1:80,
                     the default value. For a real robot, 127.0.0.1 should be replaced with the IP address
                     of the robot controller. The WAN port ethernet must be used, not the maintenance port.
    :param username: The HTTP username for the robot. Defaults to 'Default User'
    :param password: The HTTP password for the robot. Defaults to 'robotics'
    """
    def __init__(self, base_url='http://127.0.0.1:80', username=None, password=None):
        self.base_url=base_url
        if username is None:
            username = 'Default User'
        if password is None:
            password = 'robotics'
        self.auth=httpx.DigestAuth(username, password)
        self._session=httpx.AsyncClient(auth=self.auth)
        self._rmmp_session=None
        self._rmmp_session_t=None
        self._websocket = None
        self._websocket_lock = asyncio.locks.Lock()
        
    async def _do_get(self, relative_url):
        url="/".join([self.base_url, relative_url])
        if "?" in url:
            url += "&json=1"
        else:
            url += "?json=1"
        res=await self._session.get(url)
        try:            
            return self._process_response(res)
        finally:
            await res.aclose()
    

    async def _do_post(self, relative_url, payload=None):
        url="/".join([self.base_url, relative_url])
        if "?" in url:
            url += "&json=1"
        else:
            url += "?json=1"
        res=await self._session.post(url, data=payload)
        try:
            return self._process_response(res)
        finally:
            await res.aclose()

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

    async def start(self, cycle: str='asis',tasks: List[str]=['T_ROB1']):
        """
        Start one or more RAPID tasks

        :param cycle: The cycle mode of the robot. Can be `asis`, `once`, or `forever`.
        :param tasks: One or more tasks to start.
        """
        rob_tasks = await self.get_tasks()
        for t in tasks:
            if t not in rob_tasks
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
        res=await self._do_post("rw/rapid/execution?action=start", payload)

    async def activate_task(self, task: str):
        """
        Activate a RAPID task

        :param task: The name of the task to activate
        """
        payload={}
        await self._do_post(f"rw/rapid/tasks/{task}?action=activate",payload)

    async def deactivate_task(self, task: str):
        """
        Deactivate a RAPID task

        :param task: The name of the task to activate
        """
        payload={}
        await self._do_post(f"rw/rapid/tasks/{task}?action=deactivate",payload)

    async def stop(self):
        """
        Stop RAPID execution of normal tasks
        """
        payload={"stopmode": "stop"}
        res=await self._do_post("rw/rapid/execution?action=stop", payload)

    async def resetpp(self):
        """
        Reset RAPID program pointer to main in normal tasks
        """
        res=await self._do_post("rw/rapid/execution?action=resetpp")

    async def get_ramdisk_path(self) -> str:
        """
        Get the path of the RAMDISK variable on the controller

        :return: The RAMDISK path
        """
        res_json = await self._do_get("ctrl/$RAMDISK")
        return res_json["_embedded"]["_state"][0]["_value"]

    async def get_execution_state(self) -> RAPIDExecutionState:
        """
        Get the RAPID execution state

        :return: The RAPID execution state
        """
        res_json = await self._do_get("rw/rapid/execution")
        state = res_json["_embedded"]["_state"][0]
        ctrlexecstate=state["ctrlexecstate"]
        cycle=state["cycle"]
        return RAPIDExecutionState(ctrlexecstate, cycle)
    
    async def get_controller_state(self) -> str:
        """
        Get the controller state. The controller state can have the following values:

        `init`, `motoroff`, `motoron`, `guardstop`, `emergencystop`, `emergencystopreset`, `sysfail`

        RAPID can only be executed and the robot can only be moved in the `motoron` state.

        :return: The controller state
        """
        res_json = await self._do_get("rw/panel/ctrlstate")
        state = res_json["_embedded"]["_state"][0]
        return state['ctrlstate']

    async def set_controller_state(self, ctrl_state):
        payload = {"ctrl-state": ctrl_state}
        res=await self._do_post("rw/panel/ctrlstate?action=setctrlstate", payload)
    
    async def get_operation_mode(self) -> str:
        """
        Get the controller operational mode. The controller operational mode can have the following values:

        `INIT`, `AUTO_CH`, `MANF_CH`, `MANR`, `MANF`, `AUTO`, `UNDEF`

        Typical values returned by the controller are `AUTO` for auto mode, and `MANR` for manual reduced-speed mode.
        
        :return: The controller operational mode.
        """
        res_json = await self._do_get("rw/panel/opmode")        
        state = res_json["_embedded"]["_state"][0]
        return state["opmode"]
    
    async def get_digital_io(self, signal: str, network: str='Local', unit: str='DRV_1') -> int:
        """
        Get the value of a digital IO signal.

        :param signal: The name of the signal
        :param network: The network the signal is on. The default `Local` will work for most signals.
        :param unit: The drive unit of the signal. The default `DRV_1` will work for most signals.
        :return: The value of the signal. Typically 1 for ON and 0 for OFF
        """
        res_json = await self._do_get("rw/iosystem/signals/" + network + "/" + unit + "/" + signal)
        state = res_json["_embedded"]["_state"][0]["lvalue"]
        return int(state)
    
    async def set_digital_io(self, signal: str, value: Union[bool,int], network: str='Local', unit: str='DRV_1'):
        """
        Set the value of an digital IO signal.

        :param value: The value of the signal. Bool or bool convertible input
        :param signal: The name of the signal
        :param network: The network the signal is on. The default `Local` will work for most signals.
        :param unit: The drive unit of the signal. The default `DRV_1` will work for most signals.
        """
        lvalue = '1' if bool(value) else '0'
        payload={'lvalue': lvalue}
        res=await self._do_post("rw/iosystem/signals/" + network + "/" + unit + "/" + signal + "?action=set", payload)

    async def get_analog_io(self, signal: str, network: str='Local', unit: str='DRV_1') -> float:
        """
        Get the value of an analog IO signal.

        :param signal: The name of the signal
        :param network: The network the signal is on. The default `Local` will work for most signals.
        :param unit: The drive unit of the signal. The default `DRV_1` will work for most signals.
        :return: The value of the signal
        """
        res_json = await self._do_get("rw/iosystem/signals/" + network + "/" + unit + "/" + signal)
        state = res_json["_embedded"]["_state"][0]["lvalue"]
        return float(state)
    
    async def set_analog_io(self, signal: str, value: int, network: str='Local', unit: str='DRV_1'):
        """
        Set the value of an analog IO signal.

        :param value: The value of the signal
        :param signal: The name of the signal
        :param network: The network the signal is on. The default `Local` will work for most signals.
        :param unit: The drive unit of the signal. The default `DRV_1` will work for most signals.
        """
        payload={"mode": "value",'lvalue': value}
        res=await self._do_post("rw/iosystem/signals/" + network + "/" + unit + "/" + signal + "?action=set", payload)
    
    async def get_rapid_variables(self, task: str="T_ROB1") -> str:
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
        res_json = await self._do_post(f"rw/rapid/symbols?action=search-symbols", payload)
        state = res_json["_embedded"]["_state"]
        return state

    async def get_rapid_variable(self, var: str, task: str = "T_ROB1") -> str:
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
        res_json = await self._do_get("rw/rapid/symbol/data/RAPID/" + var1)
        state = res_json["_embedded"]["_state"][0]["value"]
        return state
    
    async def set_rapid_variable(self, var: str, value: str, task: str = "T_ROB1"):
        """
        Set value of a RAPID pers variable

        :param var: The pers variable name
        :param value: The new variable value encoded as a string
        :param task: The task containing the pers variable
        """
        payload={'value': value}
        if task is not None:
            var1 = f"{task}/var"
        else:
            var1 = var
        res=await self._do_post("rw/rapid/symbol/data/RAPID/" + var1 + "?action=set", payload)
        
    async def read_file(self, filename: str) -> bytes:
        """
        Read a file off the controller

        :param filename: The filename to read
        :return: The file bytes
        """
        url="/".join([self.base_url, "fileservice", filename])
        res=await self._session.get(url)
        if not res.is_success:
            raise Exception(f"File not found {filename}")
        try:            
            return res.content
        finally:
            await res.aclose()

    async def upload_file(self, filename: str, contents: bytes) -> None:
        """
        Upload a file to the controller

        :param filename: The filename to write
        :param contents: The file content bytes
        """
        url="/".join([self.base_url, "fileservice" , filename])
        res=await self._session.put(url, content=contents)
        if not res.is_success:
            raise Exception(res.reason_phrase)
        await res.aclose()

    async def delete_file(self, filename: str) -> None:
        """
        Delete a file on the controller

        :param filename: The filename to delete
        """
        url="/".join([self.base_url, "fileservice" , filename])
        res=await self._session.delete(url)
        await res.aclose()

    async def list_files(self, path: str) -> List[str]:
        """
        List files at a path on a controller

        :param path: The path to list
        :return: The filenames in the path
        """
        res_json = await self._do_get("fileservice/" + str(path) + "")
        state = res_json["_embedded"]["_state"]
        return [f["_title"] for f in state]

    async def read_event_log(self, elog: int=0) -> List[EventLogEntry]:
        """
        Read the controller event log

        :param elog: The event log id to read
        :return: The event log entries        
        """
        o=[]
        res_json = await self._do_get("rw/elog/" + str(elog) + "/?lang=en")
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

    async def get_tasks(self) -> List[TaskState]:
        """
        Get controller tasks and task state

        :return: The tasks and task state
        """
        o = {}
        res_json = await self._do_get("rw/rapid/tasks")
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

    async def get_jointtarget(self, mechunit: str="ROB_1"):
        """
        Get the current jointtarget for specified mechunit

        :param mechunit: The mechanical unit to read
        :return: The current jointtarget
        """
        res_json=await self._do_get("rw/motionsystem/mechunits/" + mechunit + "/jointtarget")
        state = res_json["_embedded"]["_state"][0]
        if not state["_type"] == "ms-jointtarget":
            raise Exception("Invalid response from controller")
        robjoint=np.array([state["rax_1"], state["rax_2"], state["rax_3"], state["rax_4"], state["rax_5"], 
            state["rax_6"]], dtype=np.float64)
        extjoint=np.array([state["eax_a"], state["eax_b"], state["eax_c"], state["eax_d"], state["eax_e"], 
            state["eax_f"]], dtype=np.float64)
     
        return JointTarget(robjoint,extjoint)
        
    async def get_robtarget(self, mechunit: str='ROB_1', tool: str='tool0', wobj: str='wobj0', coordinate: str='Base'):
        """
        Get the current robtarget (cartesian pose) for the specified mechunit

        :param mechunit: The mechanical unit to read
        :param tool: The tool to use to compute robtarget
        :param wobj: The wobj to use to compute robtarget
        :param coordinate: The coordinate system to use to compute robtarget. Can be `Base`, `World`, `Tool`, or `Wobj`
        :return: The current robtarget

        """
        res_json=await self._do_get(f"rw/motionsystem/mechunits/{mechunit}/robtarget?tool={tool}&wobj={wobj}&coordinate={coordinate}")
        state = res_json["_embedded"]["_state"][0]
        if not state["_type"] == "ms-robtargets":
            raise Exception("Invalid response from controller")
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
    
    async def get_rapid_variable_jointtarget(self, var: str, task: str = "T_ROB1"):
        """
        Get a RAPID pers variable and convert to JointTarget

        :param var: The pers variable name
        :param task: The task containing the pers variable
        :return: The pers variable encoded as a JointTarget
        """
        v = await self.get_rapid_variable(var, task)
        return self._rws_value_to_jointtarget(v)
    
    async def set_rapid_variable_jointtarget(self,var: str, value: JointTarget, task: str = "T_ROB1"):
        """
        Set a RAPID pers variable from a JointTarget

        :param var: The pers variable name
        :param value: The new variable JointTarget value
        :param task: The task containing the pers variable
        """
        rws_value=self._jointtarget_to_rws_value(value)
        await self.set_rapid_variable(var, rws_value, task)
            
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
    
    async def get_rapid_variable_jointtarget_array(self, var, task: str = "T_ROB1"):
        """
        Get a RAPID pers variable and convert to JointTarget list

        :param var: The pers variable name
        :param task: The task containing the pers variable
        :return: The pers variable encoded as a list of JointTarget
        """
        v = await self.get_rapid_variable(var, task)
        return self._rws_value_to_jointtarget_array(v)
    
    async def set_rapid_variable_jointtarget_array(self,var: str, value: List[JointTarget], task: str = "T_ROB1"):
        """
        Set a RAPID pers variable from a JointTarget list

        :param var: The pers variable name
        :param value: The new variable JointTarget value
        :param task: The task containing the pers variable
        """
        rws_value=self._jointtarget_array_to_rws_value(value)
        await self.set_rapid_variable(var, rws_value, task)

    async def get_rapid_variable_num(self, var: str, task: str = "T_ROB1"):
        """
        Get a RAPID pers variable and convert to float

        :param var: The pers variable name
        :param task: The task containing the pers variable
        :return: The pers variable encoded as a float
        """
        return float(await self.get_rapid_variable(var,task))
    
    async def set_rapid_variable_num(self, var: str, val: float, task: str = "T_ROB1"):
        """
        Set a RAPID pers variable from a float

        :param var: The pers variable name
        :param value: The new variable float value
        :param task: The task containing the pers variable
        """
        await self.set_rapid_variable(var, str(val), task)
        
    async def get_rapid_variable_num_array(self, var, task: str = "T_ROB1") -> np.ndarray:
        """
        Get a RAPID pers variable float array

        :param var: The pers variable name
        :param task: The task containing the pers variable
        :return: The variable value as an array
        """
        val1=await self.get_rapid_variable(var,task)
        m=re.match("^\\[([^\\]]*)\\]$", val1)
        val2=m.groups()[0].strip()
        return np.fromstring(val2,sep=',')
    
    async def set_rapid_variable_num_array(self, var: str, val: List[float], task: str = "T_ROB1"):
        """
        Set a RAPID pers variable from a float list or array

        :param var: The pers variable name
        :param value: The new variable float array value
        :param task: The task containing the pers variable
        """
        await self.set_rapid_variable(var, "[" + ','.join([str(s) for s in val]) + "]", task)

    async def read_ipc_message(self, queue_name: str, timeout: float=0) -> List[IpcMessage]:
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
        
        res_json=await self._do_get("rw/dipc/" + queue_name + "/?action=dipc-read" + timeout_str)
        for state in res_json["_embedded"]["_state"]:
            if not state["_type"] == "dipc-read-li":
                raise Exception("Invalid response from controller")
     
            o.append(IpcMessage(state["dipc-data"], state["dipc-userdef"],
                state["dipc-msgtype"], state["dipc-cmd"], state["queue-name"]))
            
            #o.append(RAPIDEventLogEntry(msg_type,code,tstamp,args,title,desc,conseqs,causes,actions))
        return o
    
    async def send_ipc_message(self, target_queue: str, data: str, queue_name: str, cmd: int=111, userdef: int=1, msgtype: int=1 ):
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
        res=await self._do_post("rw/dipc/" + target_queue + "?action=dipc-send", payload)
    
    async def get_ipc_queue(self, queue_name):
        """
        Get the IPC queue

        :param queue_name: The name of the queue
        """
        res=await self._do_get("rw/dipc/" + queue_name + "?action=dipc-read")
        return res
    
    async def try_create_ipc_queue(self, queue_name: str, queue_size: int=4440, max_msg_size: int=444):
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
            await self._do_post("rw/dipc?action=dipc-create", payload)
            return True
        except ABBException as e:
            if e.code==-1073445879:
                return False
            raise
    
    async def request_rmmp(self, timeout: float=5):
        """
        Request Remote Mastering. Required to alter pers variables in manual control mode. The teach pendant
        will prompt to enable remote mastering, and the user must confirm. Once remote mastering is enabled,
        poll_rmmp() must be executed periodically to maintain rmmp.

        :param timeout: The request timeout in seconds
        """
        t1=time.time()
        await self._do_post('users/rmmp?json=1', {'privilege': 'modify'})
        while time.time() - t1 < timeout:
            
            res_json=self._do_get('users/rmmp/poll?json=1')
            state = res_json["_embedded"]["_state"][0]
            if not state["_type"] == "user-rmmp-poll":
                raise Exception("Invalid response from controller")
            status = state["status"]
            if status=="GRANTED":
                await self.poll_rmmp()
                return
            elif status!="PENDING":
                raise Exception("User did not grant remote access")                               
            await asyncio.sleep(0.25)
        raise Exception("User did not grant remote access")
    
    async def poll_rmmp(self):
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
            self._rmmp_session=httpx.AsyncClient()
            self._rmmp_session_t=time.time()            
            
            for c in self._session.cookies:
                self._rmmp_session.cookies.set(c) 
        else:
            if time.time() - self._rmmp_session_t > 30:
                old_rmmp_session=self._rmmp_session
                rmmp_session=httpx.AsyncClient()
                
                for c in self._session.cookies:
                    rmmp_session.cookies.set(c)
        
        rmmp_session=self._rmmp_session        
                
        res=await rmmp_session.get(url)
        res_json=self._process_response(res)
        state = res_json["_embedded"]["_state"][0]
        if not state["_type"] == "user-rmmp-poll":
            raise Exception("Invalid response from controller")
                
        if old_rmmp_session is not None:
            self._rmmp_session=rmmp_session
            self._rmmp_session_t=time.time()
            try:
                old_rmmp_session.aclose()
            except:
                pass
       
        return state["status"] == "GRANTED"

    async def logout(self):
        try:
            res=await self._do_get("logout")
        finally:
            try:
                await self._session.aclose()
            except Exception:
                pass

    async def subscribe(self, resources: List[SubscriptionResourceRequest]):
        """
        Create subscription that will receive real-time updates from the controller. handler will be called
        with the new values. RWS subscriptions are relatively slow, with delays in the hundreds of milliseconds. 
        Use EGM for faster real-time updates.

        See :meth:`abb_robot_client.RWS.subscribe()` for more information on `resources` parameter.

        The asyncio version of subscribe() returns an async generator. Use `async for` to receive events.

        :param resources: Controller resources to subscribe
        :param handler: Callable to call on resource events
        :return: Generator to use with `async for`.
        """
        self._init_subscription_convert_message()

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
        res1=await self._session.post(url, data=payload)
        try:
            res=self._process_response(res1)
        finally:
            await res1.aclose()

        if not res1.status_code == 201:
            raise Exception("Subscription creation failed")

        m = re.search(r'<a href="ws:\/\/.*(\/poll\/\d+)" rel="self">', res1.content.decode("ascii"))
        if not m:
            raise Exception("Invalid response from controller")
        ws_url = self.base_url.replace("http:","ws:") + m.group(1)
        
        cookie = f"-http-session-={self._session.cookies['-http-session-']}; ABBCX={self._session.cookies['ABBCX']}"
        
        header={'Cookie': cookie}

        async with self._websocket_lock:
            if not self._websocket is None: 
                raise Exception("Subscription already created")

            self._websocket = await websockets.connect(
                ws_url, 
                extra_headers = header,
                subprotocols=['robapi2_subscription']
            )

        try:
            while True:
                message = await self._websocket.recv()
                conv_message = self._convert_subscription_message(message)
                if conv_message is not None:
                    yield conv_message
        finally:
            await self._websocket.close()            
    
    def _init_subscription_convert_message(self):
        self._signal_re = re.compile(r'<a\s+href="/rw/iosystem/signals/([^"]+);state"\s+rel="self"/?>.*<span\s+class="lvalue">([^<]+)<')
        self._pers_re = re.compile(r'<a\s+href="/rw/rapid/symbol/data/RAPID/([^"]+);value"\s+rel="self"/?>.*<span\s+class="value">([^<]+)<')
        self._elog_re = re.compile(r'<a\s+href="/rw/elog/0/([^"]+)"\s+rel="self"/?>.*<span\s+class="seqnum">([^<]+)<')
        self._exec_re = re.compile(r'<a\s+href="/rw/rapid/execution;ctrlexecstate"\s+rel="self"/?>.*<span\s+class="ctrlexecstate">([^<]+)<')
        self._opmode_re = re.compile(r'<a\s+href="/rw/panel/opmode"\s+rel="self"/?>.*<span\s+class="opmode">([^<]+)<')
        self._ctrl_re = re.compile(r'<a\s+href="/rw/panel/ctrlstate"\s+rel="self"/?>.*<span\s+class="ctrlstate">([^<]+)<')
        self._ipc_re = re.compile(r'<a\s+href="/rw/dipc/([^"]*)".*<span\s+class="dipc-data">([^<]+)<.*<span\s+class="dipc-userdef">([^<]+)<')

    def _convert_subscription_message(self, message):
        if 'li class="ios-signalstate-ev"' in message:
            m = self._signal_re.search(message)
            if m is not None:
                sig_path = m.group(1).split('/')
                return Signal(sig_path[-1], m.group(2))
        elif 'li class="rap-data"' in message:
            m = self._pers_re.search(message)
            if m is not None:
                pers_path = m.group(1).split('/')
                if len(pers_path) == 1:            
                    return VariableValue(pers_path[0], m.group(2))
                else:
                    return VariableValue(pers_path[0], m.group(2), pers_path[-1])
        elif 'li class="elog-message-ev"' in message:
            m = self._elog_re.search(message)
            if m is not None:
                return EventLogEntryEvent(m.group(2))
        elif 'li class="rap-ctrlexecstate-ev"' in message:
            m = self._exec_re.search(message)
            if m is not None:
                return RAPIDExecutionState(m.group(1),'')
        elif 'li class="pnl-opmode-ev"' in message:
            m = self._opmode_re.search(message)
            if m is not None:
                return OperationalMode(m.group(1))
        elif 'li class="pnl-ctrlstate-ev"' in message:
            m = self._ctrl_re.search(message)
            if m is not None:
                return ControllerState(m.group(1))
        elif 'li class="dipc-msg-ev"' in message:
            m = self._ipc_re.search(message)
            if m is not None:
                return IpcMessage(queue_name=m.group(1), data=m.group(2), userdef=m.group(3), msgtype="", cmd="")
        return UnknownSubscriptionMessage(message)

class UnknownSubscriptionMessage(NamedTuple):
    """Contents of an unknown subscription resource type"""
    xml: str
    """Raw xml returned by controller"""
