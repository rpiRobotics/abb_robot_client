from abb_robot_client import egm, rws
from contextlib import closing
import time
import numpy as np

_robot_url = "http://localhost:80"
# _robot_url = "https://localhost:443"

def test_controller_state():
    c = rws.RWS(_robot_url)
    with closing(c):        
        c.get_execution_state()
        c.get_controller_state()
        c.get_operation_mode()

def test_rws_tasks():
    c = rws.RWS(_robot_url)
    with closing(c):
        try:
            c.stop()
            time.sleep(0.5)
        except:
            pass

        c.set_analog_io("mode",0)
        c.resetpp()
        c.start("forever", ["T_ROB1"])
        time.sleep(0.1)
        assert c.get_execution_state().ctrlexecstate == "running"
        assert c.get_execution_state().cycle == "forever"
        time.sleep(1)
        c.stop()
        assert c.get_execution_state().ctrlexecstate == "stopped"

        task_state = c.get_tasks()

        c.deactivate_task("T_ROB1")
        c.activate_task("T_ROB1")

def test_signal():
    c = rws.RWS(_robot_url)
    with closing(c):
        c.set_digital_io("test_digital_io1", 1)
        assert c.get_digital_io("test_digital_io1") == 1
        c.set_digital_io("test_digital_io1", 0)
        assert c.get_digital_io("test_digital_io1") == 0

        c.set_digital_io("test_digital_io2", 1,  "DeviceNet", "d651")
        assert c.get_digital_io("test_digital_io2", "DeviceNet", "d651") == 1
        c.set_digital_io("test_digital_io2", 0, "DeviceNet", "d651")
        assert c.get_digital_io("test_digital_io2", "DeviceNet", "d651") == 0

        c.set_analog_io("test_analog_io1", 14.285)
        assert abs(c.get_analog_io("test_analog_io1") - 14.285) < 0.001
        c.set_analog_io("test_analog_io1", 0)
        assert abs(c.get_analog_io("test_analog_io1") - 0) < 0.001

        c.set_analog_io("test_analog_io2", 8.562, "DeviceNet", "d651")
        assert abs(c.get_analog_io("test_analog_io2", "DeviceNet", "d651") - 8.562) < 0.001
        c.set_analog_io("test_analog_io2", 0, "DeviceNet", "d651")
        assert abs(c.get_analog_io("test_analog_io2", "DeviceNet", "d651") - 0) < 0.001

def test_rapid_vars():
    c = rws.RWS(_robot_url)
    with closing(c):
        # TODO: get_rapid_variables() is not implemented
        # rapid_vars = c.getf_rapid_variables("T_ROB1")
        # assert "test_var_str" in rapid_vars
        c.set_rapid_variable("test_var_str", "\"Hello World!\"", "T_ROB1")
        assert c.get_rapid_variable("test_var_str", "T_ROB1") == "\"Hello World!\""
        c.set_rapid_variable("test_var_num", str(123.456), "T_ROB1/Module1")
        assert abs(float(c.get_rapid_variable("test_var_num", "T_ROB1/Module1")) - 123.456) < 0.001

def test_files():
    c = rws.RWS(_robot_url)
    with closing(c):
        ramdisk_path = c.get_ramdisk_path()
        data = "Hello World from file!".encode("utf-8")
        c.upload_file(ramdisk_path + "/test_file.txt", data)
        file_list = c.list_files(ramdisk_path)
        assert "test_file.txt" in file_list
        assert c.read_file(ramdisk_path + "/test_file.txt") == data
        c.delete_file(ramdisk_path + "/test_file.txt")
        file_list = c.list_files(ramdisk_path)
        assert "test_file.txt" not in file_list

def test_evtlog():
    c = rws.RWS(_robot_url)
    with closing(c):
        evts = c.read_event_log()
        print(evts)

def test_current_targets():
    c = rws.RWS(_robot_url)
    with closing(c):
        c.get_jointtarget("ROB_1")
        c.get_robtarget("ROB_1")
        c.get_robtarget("ROB_1", "tool0", "wobj0", "Base")

def test_speed_ratio():
    c = rws.RWS(_robot_url)
    with closing(c):
        c.get_speedratio()
        c.set_speedratio(50)
        assert c.get_speedratio() == 50
        c.set_speedratio(100)
        assert c.get_speedratio() == 100

def test_egm_position_command():
    c = rws.RWS(_robot_url)
    e = egm.EGM()
    with closing(c), closing(e):
        try:
            c.stop()
            time.sleep(0.5)
        except:
            pass

        c.set_analog_io("mode",1)
        c.resetpp()
        c.start("once", ["T_ROB1"])
        time.sleep(0.1)
        assert c.get_execution_state().ctrlexecstate == "running"
        time.sleep(1)

        recv_count = 0
        loop_count = 0
        while True:
            try:
                res, _ = e.receive_from_robot(0.1)
                if res:
                    recv_count += 1
                    if recv_count > 5:
                        break
            except:
                pass
            loop_count += 1
            assert loop_count < 10

        
        joints = [21, -10, -20, 30, 40, 50]
        external_joints = [90,100]
        joints_speed = [10,10,10,10,10,10]
        external_joints_speed = [10,10]
        rapid_to_robot = [88,99]
        e.send_to_robot(joints, joints_speed, external_joints, external_joints_speed, rapid_to_robot)
        time.sleep(3)
        e.send_to_robot(joints)
        time.sleep(2)
        while True:
            res, _ = e.receive_from_robot()
            if not res:
                break
        res, egm_state = e.receive_from_robot(1)
        assert res
        np.testing.assert_allclose(egm_state.joint_angles, joints, atol=0.1)
        c.set_digital_io("stop_egm", 1)
        time.sleep(3)
        assert c.get_execution_state().ctrlexecstate == "stopped"

def test_egm_pose_command():
    c = rws.RWS(_robot_url)
    e = egm.EGM()
    with closing(c), closing(e):
        try:
            c.stop()
            time.sleep(0.5)
        except:
            pass

        c.set_analog_io("mode",2)
        c.resetpp()
        c.start("once", ["T_ROB1"])
        time.sleep(0.1)
        assert c.get_execution_state().ctrlexecstate == "running"
        time.sleep(1)

        recv_count = 0
        loop_count = 0
        while True:
            try:
                res, _ = e.receive_from_robot(0.1)
                if res:
                    recv_count += 1
                    if recv_count > 5:
                        break
            except:
                pass
            loop_count += 1
            assert loop_count < 10

        
        pose_lin = [535,181,625]
        pose_quat = [0.704416,0.1227878,0.6963642,0.0616284]
        pose_speed = [0.1,0.11,0.12,0.13,0.14,0.15]
        external_joints = [90,100]
        external_joints_speed = [10,10]
        rapid_to_robot = [88,99]
        e.send_to_robot_cart(pose_lin, pose_quat, pose_speed, external_joints, external_joints_speed, rapid_to_robot)
        time.sleep(0.1)
        while True:
            res, _ = e.receive_from_robot()
            if not res:
                break
        e.send_to_robot_cart(pose_lin, pose_quat)
        time.sleep(2)
        while True:
            res, _ = e.receive_from_robot()
            if not res:
                break
        res, egm_state = e.receive_from_robot(0.5)
        assert res
        np.testing.assert_allclose(egm_state.cartesian[0], pose_lin, atol=0.1)
        np.testing.assert_allclose(egm_state.cartesian[1], pose_quat, atol=0.1)
        c.set_digital_io("stop_egm", 1)
        time.sleep(3)
        assert c.get_execution_state().ctrlexecstate == "stopped"

def test_egm_path_corr():
    c = rws.RWS(_robot_url)
    e = egm.EGM()
    with closing(c), closing(e):
        try:
            c.stop()
            time.sleep(0.5)
        except:
            pass

        c.set_analog_io("mode",3)
        c.resetpp()
        c.start("once", ["T_ROB1"])
        time.sleep(0.1)
        assert c.get_execution_state().ctrlexecstate == "running"
        time.sleep(1)

        recv_count = 0
        loop_count = 0
        while True:
            try:
                res, _ = e.receive_from_robot(0.1)
                if res:
                    recv_count += 1
                    if recv_count > 5:
                        break
            except:
                pass
            loop_count += 1
            assert loop_count < 10

        for i in range(10):
            e.send_to_robot_path_corr([10,20,0.0], 1)
            time.sleep(0.5)
            e.send_to_robot_path_corr([-10,-20,0.0],1)
            time.sleep(0.5)
        
        time.sleep(0.1)
        c.stop()