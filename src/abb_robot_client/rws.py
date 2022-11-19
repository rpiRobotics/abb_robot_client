import struct
import numpy as np
import io
import requests
import datetime
import re
import time

from typing import Callable, NamedTuple, Any, List, Union
from enum import IntEnum

class ABBException(Exception):
    def __init__(self, message, code):
        super(ABBException, self).__init__(message)
        self.code=code

class RAPIDExecutionState(NamedTuple):
    ctrlexecstate: Any
    cycle: Any

class RAPIDEventLogEntry(NamedTuple):
    seqnum: int
    msgtype: int
    code: int 
    tstamp: datetime.datetime 
    args: List[Any]
    title: str
    desc: str
    conseqs: str
    causes: str
    actions: str

class RAPIDTaskState(NamedTuple):
    name: str
    type_: str
    taskstate: str
    excstate: str
    active: bool
    motiontask: bool

class JointTarget(NamedTuple):
    robax: np.array
    extax: np.array

class RobTarget(NamedTuple):
    trans: np.array
    rot: np.array
    robconf: np.array
    extax: np.array


class IpcMessage(NamedTuple):
    data: str
    userdef: str
    msgtype: str
    cmd: str

class Signal(NamedTuple):
    name: str
    lvalue: str

class RWS:
    def __init__(self, base_url='http://127.0.0.1:80', username='Default User', password='robotics'):
        self.base_url=base_url
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

    def start(self, cycle: str='asis',tasks: List[str]=['T_ROB1']) -> None:

        rob_tasks = self.get_tasks()
        for t in tasks:
            assert t in rob_tasks, f"Cannot start unknown task {t}"

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

    def activate_task(self, task: str) -> None:
        payload={}
        self._do_post(f"rw/rapid/tasks/{task}?action=activate",payload)

    def deactivate_task(self, task: str) -> None:
        payload={}
        self._do_post(f"rw/rapid/tasks/{task}?action=deactivate",payload)

    def stop(self) -> None:
        payload={"stopmode": "stop"}
        res=self._do_post("rw/rapid/execution?action=stop", payload)

    def resetpp(self) -> None:
        res=self._do_post("rw/rapid/execution?action=resetpp")

    def get_ramdisk_path(self) -> str:
        res_json = self._do_get("ctrl/$RAMDISK")
        return res_json["_embedded"]["_state"][0]["_value"]

    def get_execution_state(self) -> RAPIDExecutionState:
        res_json = self._do_get("rw/rapid/execution")
        state = res_json["_embedded"]["_state"][0]
        ctrlexecstate=state["ctrlexecstate"]
        cycle=state["cycle"]
        return RAPIDExecutionState(ctrlexecstate, cycle)
    
    def get_controller_state(self) -> str:
        res_json = self._do_get("rw/panel/ctrlstate")
        state = res_json["_embedded"]["_state"][0]
        return state['ctrlstate']
    
    def get_operation_mode(self) -> str:
        res_json = self._do_get("rw/panel/opmode")        
        state = res_json["_embedded"]["_state"][0]
        return state["opmode"]
    
    def get_digital_io(self, signal: str, network: str='Local', unit: str='DRV_1') -> int:
        res_json = self._do_get("rw/iosystem/signals/" + network + "/" + unit + "/" + signal)
        state = res_json["_embedded"]["_state"][0]["lvalue"]
        return int(state)
    
    def set_digital_io(self, signal: str, value: Union[bool,int], network: str='Local', unit: str='DRV_1') -> None:
        lvalue = '1' if bool(value) else '0'
        payload={'lvalue': lvalue}
        res=self._do_post("rw/iosystem/signals/" + network + "/" + unit + "/" + signal + "?action=set", payload)

    def get_analog_io(self, signal: str, network: str='Local', unit: str='DRV_1') -> int:
        res_json = self._do_get("rw/iosystem/signals/" + network + "/" + unit + "/" + signal)
        state = res_json["_embedded"]["_state"][0]["lvalue"]
        return int(state)
    
    def set_analog_io(self, signal: str, value: int, network: str='Local', unit: str='DRV_1') -> None:
        payload={"mode": "value",'lvalue': value}
        res=self._do_post("rw/iosystem/signals/" + network + "/" + unit + "/" + signal + "?action=set", payload)
    
    def get_rapid_variable(self, var: str) -> str:
        res_json = self._do_get("rw/rapid/symbol/data/RAPID/T_ROB1/" + var)
        state = res_json["_embedded"]["_state"][0]["value"]
        return state
    
    def set_rapid_variable(self, var: str, value: str):
        payload={'value': value}
        res=self._do_post("rw/rapid/symbol/data/RAPID/T_ROB1/" + var + "?action=set", payload)
        
    def read_file(self, filename: str) -> bytes:
        url="/".join([self.base_url, "fileservice", filename])
        res=self._session.get(url, auth=self.auth)
        try:            
            return res.content
        finally:
            res.close()

    def upload_file(self, filename: str, contents: bytes) -> None:
        url="/".join([self.base_url, "fileservice" , filename])
        res=self._session.put(url, contents, auth=self.auth)
        assert res.ok, res.reason
        res.close()

    def delete_file(self, filename: str) -> None:
        url="/".join([self.base_url, "fileservice" , filename])
        res=self._session.delete(url, auth=self.auth)
        res.close()

    def read_event_log(self, elog: int=0) -> List[RAPIDEventLogEntry]:
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
            
            o.append(RAPIDEventLogEntry(seqnum,msg_type,code,tstamp,args,title,desc,conseqs,causes,actions))
        return o

    def get_tasks(self) -> List[RAPIDTaskState]:
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

            o[name]=RAPIDTaskState(name,type_,taskstate,excstate,active,motiontask)
        
        return o

    def get_jointtarget(self, mechunit="ROB_1"):
        res_json=self._do_get("rw/motionsystem/mechunits/" + mechunit + "/jointtarget")
        state = res_json["_embedded"]["_state"][0]
        assert state["_type"] == "ms-jointtarget"
        robjoint=np.array([state["rax_1"], state["rax_2"], state["rax_3"], state["rax_4"], state["rax_5"], 
            state["rax_6"]], dtype=np.float64)
        extjoint=np.array([state["eax_a"], state["eax_b"], state["eax_c"], state["eax_d"], state["eax_e"], 
            state["eax_f"]], dtype=np.float64)
     
        return JointTarget(robjoint,extjoint)
        
    def get_robtarget(self, mechunit='ROB_1', tool='tool0', wobj='wobj0', coordinate='Base'):
        res_json=self._do_get(f"rw/motionsystem/mechunits/{mechunit}/robtarget?tool={tool}&wobj={wobj}&coordinate={coordinate}")
        state = res_json["_embedded"]["_state"][0]
        assert state["_type"] == "ms-robtargets"
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
        assert np.shape(val[0]) == (6,)
        assert np.shape(val[1]) == (6,)
        robax=','.join([format(x, '.4f') for x in np.rad2deg(val[0])])
        extax=','.join([format(x, '.4f') for x in np.rad2deg(val[1])])
        rws_value="[[" + robax + "],[" + extax + "]]"
        return rws_value
    
    def get_rapid_variable_jointtarget(self, var):
        v = self.get_rapid_variable(var)
        return self._rws_value_to_jointtarget(v)
    
    def set_rapid_variable_jointtarget(self,var,value):
        rws_value=self._jointtarget_to_rws_value(value)
        self.set_rapid_variable(var, rws_value)
            
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
    
    def get_rapid_variable_jointtarget_array(self, var):
        v = self.get_rapid_variable(var)
        return self._rws_value_to_jointtarget_array(v)
    
    def set_rapid_variable_jointtarget_array(self,var,value):
        rws_value=self._jointtarget_array_to_rws_value(value)
        self.set_rapid_variable(var, rws_value)

    def get_rapid_variable_num(self, var):
        return float(self.get_rapid_variable(var))
    
    def set_rapid_variable_num(self, var, val):
        self.set_rapid_variable(var, str(val))
        
    def get_rapid_variable_num_array(self, var):
        val1=self.get_rapid_variable(var)
        m=re.match("^\\[([^\\]]*)\\]$", val1)
        val2=m.groups()[0].strip()
        return np.fromstring(val2,sep=',')
    
    def set_rapid_variable_num_array(self, var, val):
        self.set_rapid_variable(var, "[" + ','.join([str(s) for s in val]) + "]")

    def read_ipc_message(self, queue_name, timeout=0):
        
        o=[]
        
        timeout_str=""
        if timeout > 0:
            timeout_str="&timeout=" + str(timeout)
        
        res_json=self._do_get("rw/dipc/" + queue_name + "/?action=dipc-read" + timeout_str)
        for state in res_json["_embedded"]["_state"]:
            assert state["_type"] == "dipc-read-li"
     
            o.append(IpcMessage(state["dipc-data"], state["dipc-userdef"],
                state["dipc-msgtype"], state["dipc-cmd"]))
            
            #o.append(RAPIDEventLogEntry(msg_type,code,tstamp,args,title,desc,conseqs,causes,actions))
        return o
    
    def send_ipc_message(self, target_queue, data, queue_name, cmd=111, userdef=1, msgtype=1 ):
        payload={"dipc-src-queue-name": queue_name, "dipc-cmd": str(cmd), "dipc-userdef": str(userdef), \
                 "dipc-msgtype": str(msgtype), "dipc-data": data}
        res=self._do_post("rw/dipc/" + target_queue + "?action=dipc-send", payload)
    
    def get_ipc_queue(self, queue_name):
        res=self._do_get("rw/dipc/" + queue_name + "?action=dipc-read")
        return res
    
    def try_create_ipc_queue(self, queue_name, queue_size=4440, max_msg_size=444):
        try:
            payload={"dipc-queue-name": queue_name, "dipc-queue-size": str(queue_size), "dipc-max-msg-size": str(max_msg_size)}
            self._do_post("rw/dipc?action=dipc-create", payload)
            return True
        except ABBException as e:
            if e.code==-1073445879:
                return False
            raise
    
    def request_rmmp(self, timeout=5):
        t1=time.time()
        self._do_post('users/rmmp?json=1', {'privilege': 'modify'})
        while time.time() - t1 < timeout:
            
            res_json=self._do_get('users/rmmp/poll?json=1')
            state = res_json["_embedded"]["_state"][0]
            assert state["_type"] == "user-rmmp-poll"
            status = state["status"]
            if status=="GRANTED":
                self.poll_rmmp()
                return
            elif status!="PENDING":
                raise Exception("User did not grant remote access")                               
            time.sleep(0.25)
        raise Exception("User did not grant remote access")
    
    def poll_rmmp(self):
        
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
        assert state["_type"] == "user-rmmp-poll"
                
        if old_rmmp_session is not None:
            self._rmmp_session=rmmp_session
            self._rmmp_session_t=time.time()
            try:
                old_rmmp_session.close()
            except:
                pass
       
        return state["status"] == "GRANTED"


