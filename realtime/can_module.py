from email import message
import cantools
import can
import threading
from collections import deque
from gps_module import GPS
import numpy as np

DRIVE =  5
PARKING = 0

CONTROL_CMD = 1
DRIVING_CMD = 2

VEHICLE_INFO_1 = 3
VEHICLE_INFO_2 = 4

CYCLE_FPS = 50

dt = 0.02


class CAN:
    def __init__(self, destination = None):
        # CAN
        self.db = cantools.database.load_file('SantaFe.dbc')
        self.bus = can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=500000)
        # self.bus = can.interfaces.pcan.PcanBus(channel='PCAN1_USBBUS1', bitrate=500000)

        self.vehicle_info_1_msg = self.db.get_message_by_name('Vehicle_Info_1')
        self.vehicle_info_2_msg = self.db.get_message_by_name('Vehicle_Info_2')
        self.info_1_dict = {"APS_Feedback":0, "Brake_ACT_Feedback":0,"Gear_Shift_Feedback":0, 
                               "Steering_Angle_Feedback":0, "Switch_State":0}
        self.info_2_dict = {"Override_Feedback":0, "Vehicle_Speed":0, "Turn_Sig_Feed":0,
                               "APS_Feed":0, "BPS_Feed":0}

        self.control_cmd_msg = self.db.get_message_by_name('Control_CMD') 
        self.driving_cmd_msg = self.db.get_message_by_name('Driving_CMD')
        self.control_cmd_dict = {'Override_Off':0,'Alive_Count':0,'Angular_Speed_CMD':30}  
        self.driving_cmd_dict = {'Accel_CMD':650,'Brake_CMD':0,'Steering_CMD':0,'Gear_Shift_CMD':5, 'Reserved':0}     

        self.destination = destination
        if self.destination is None:
            self.destination = self.gps.position()

        self.max_speed = 25  # km/h

        self.cur_dist = 0

        self.gps = GPS()
    
    def get_feedback(self):
        th1 = threading.Thread(target=self.get_vehicle_info_1)
        th2 = threading.Thread(target=self.get_vehicle_info_2)

        th1.start()
        th2.start()
        th1.join()
        th2.join()
    
    def send_control(self):
        th1 = threading.Thread(target=self.send_control_cmd)
        th2 = threading.Thread(target=self.send_driving_cmd)

        th1.start()
        th2.start()
        th1.join()
        th2.join()

    @staticmethod
    def _scaler(old_value, old_min, old_max, new_min, new_max):
        return ((old_value - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min

    def start_autopilot(self):
        while self.info_1_dict["Override_Feedback"] != 1:
            self.send_control()

    def change_gear(self, gear):
        if self.info_2_dict["Vehicle_Speed"] == 0 and self.driving_cmd_dict["Break_ACT_Feedback"] >= 30000:
            self.driving_cmd_dict["Gear_Shift_CMD"] = gear
            self.send_control()
            return
        else:
            self.driving_cmd_dict["Break_CMD"] = 16500
            self.send_control()
            self.change_gear(gear)
    
    def done(self):
        self.control_cmd_dict["Override_Off"] = 1
        self.driving_cmd_dict["Break_CMD"] = 17000
        self.send_control()

    def reached_destination(self):
        cur_position = self.gps.position()
        if abs(cur_position[0] - self.destination[0]) + abs(cur_position[1] - self.destination[1]) < 0.000015:
            return True
        return False

    def control(self):
        if self.info_1_dict["Override_Feedback"] != 1:
            return True

        if not self.reached_destination:
            if self.info_2_dict["Vehicle_Speed"] < self.max_speed:
                self.driving_cmd_dict["Accel_CMD"] = 1000
            else:
                self.driving_cmd_dict["Accel_CMD"] = 0
        else:
            if self.driving_cmd_dict["Accel_CMD"] != 0:
                self.driving_cmd_dict["Accel_CMD"] = 0
            self.driving_cmd_dict["Break_CMD"] = max(self.driving_cmd_dict["Break_CMD"] + 100, 16999)

            if self.info_2_dict["Vehicle_Speed"] == 0:
                return True
        self.send_control()
        return False

    def get_vehicle_info_1(self):
        try:
            msg = self.bus.recv(0.1)  # wait 0.5 sec to get msg then raise error
            data = self.db.decode_message(msg.arbitration_id, msg.data)
            if msg.arbitration_id == self.vehicle_info_1_msg.frame_id:
                self.info_1_dict = data
                if self.info_1_dict["Switch_State"] == 30:
                    self.auto_stanby = True
                else:
                    self.auto_stanby = False
        except:
            return 0

    def get_vehicle_info_2(self):
        try:
            msg = self.bus.recv(0.1)  # wait 0.5 sec to get msg then raise error
            data = self.db.decode_message(msg.arbitration_id, msg.data)
            if msg.arbitration_id == self.vehicle_info_2_msg.frame_id:
                self.info_2_dict = data
                if self.info_2_dict["Override_Feedback"] != 0:
                    return 0
        except:
            return 0

    def send_control_cmd(self):
        # print("_send_command")
        self.control_cmd_dict['Alive_Count'] += 1
        self.control_cmd_dict['Alive_Count'] %= 256
        data = self.control_cmd_msg.encode(self.control_cmd_dict)
        message = can.Message(arbitration_id=self.control_cmd_msg.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
    

    def send_driving_cmd(self):
        data = self.driving_cmd_msg.encode(self.driving_cmd_dict)
        message = can.Message(arbitration_id=self.driving_cmd_msg.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
    




