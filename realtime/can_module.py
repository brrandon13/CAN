import cantools
import can
import threading
from pygame import time
from geopy import distance
from controller import LongitudinalController



DRIVE =  5
PARKING = 0

CONTROL_CMD = 1
DRIVING_CMD = 2

VEHICLE_INFO_1 = 3
VEHICLE_INFO_2 = 4

CYCLE_FPS = 50

dt = 0.02

global lat, lon


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

        self.max_speed = 7  # m/s 
        self.clock = time.Clock()

        self.cur_dist = 0

        self.controller = LongitudinalController(self.destination, self.max_speed)
    
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
        print("starting autopilot ...", end=" ")
        while self.info_1_dict["Override_Feedback"] != 1:
            self.clock.tick_busy_loop(50)
            self.send_control()
        print("done.")

    def change_gear(self, gear):
        print("changing gear ...", end=" ")
        while self.info_1_dict["Gear_Shift_Feedback"] != gear:
            self.clock.tick_busy_loop(50)
            if self.info_2_dict["Vehicle_Speed"] == 0 and self.driving_cmd_dict["Brake_ACT_Feedback"] >= 20000:
                self.driving_cmd_dict["Gear_Shift_CMD"] = gear
            else:
                self.driving_cmd_dict["Brake_CMD"] += 1000
            self.send_control()
        print("done")
    
    def reached_destination(self, cur_position):
        if distance.distance(self.destination, cur_position) <= 0.001:
            print("reached destination. stopping...")
            return True
        return False

    def done(self):
        print("driving finished")
        self.control_cmd_dict["Override_Off"] = 1
        self.send_control()

    def control(self, position):
        if self.info_1_dict["Override_Feedback"] != 1:
            return True
        cur_speed = self.info_2_dict["Vehicle_Speed"]
        accel = self.controller.run_step(cur_speed, position)

        if accel > 0:
            self.driving_cmd_dict["Accel_CMD"] = accel * 100 + 650
        else:
            self.driving_cmd_dict["Accel_CMD"] = 0
            self.driving_cmd_dict["Brake_CMD"] = -accel * 1000

        self.send_control()

    def get_vehicle_info_1(self):
        try:
            msg = self.bus.recv(0.1)  # wait 0.5 sec to get msg then raise error
            data = self.db.decode_message(msg.arbitration_id, msg.data)
            if msg.arbitration_id == self.vehicle_info_1_msg.frame_id:
                self.info_1_dict = data
        except:
            return 0

    def get_vehicle_info_2(self):
        try:
            msg = self.bus.recv(0.1)  # wait 0.5 sec to get msg then raise error
            data = self.db.decode_message(msg.arbitration_id, msg.data)
            if msg.arbitration_id == self.vehicle_info_2_msg.frame_id:
                self.info_2_dict = data
        except:
            return 0

    def send_control_cmd(self):
        # print("_send_command")
        self.control_cmd_dict['Alive_Count'] = (self.control_cmd_dict['Alive_Count'] + 1) % 256
        data = self.control_cmd_msg.encode(self.control_cmd_dict)
        message = can.Message(arbitration_id=self.control_cmd_msg.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
    

    def send_driving_cmd(self):
        data = self.driving_cmd_msg.encode(self.driving_cmd_dict)
        message = can.Message(arbitration_id=self.driving_cmd_msg.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
    




