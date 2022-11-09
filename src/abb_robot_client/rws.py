import struct
import numpy as np
import io
import requests
import datetime

from typing import Callable, NamedTuple, Any, List, Union

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

class RWS:
    def __init__(self, base_url='http://127.0.0.1:80', username='Default User', password='robotics'):
        self.base_url=base_url
        self.auth=requests.auth.HTTPDigestAuth(username, password)
        self._session=requests.Session()
        
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