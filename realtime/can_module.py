import cantools
import can
import threading

CONTROL_CMD = 1
DRIVING_CMD = 2

VEHICLE_INFO_1 = 3
VEHICLE_INFO_2 = 4

CYCLE_FPS = 50


class CAN:
    def __init__(self):
        # CAN
        self.db = cantools.database.load_file('SantaFe.dbc')
        self.bus = can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=500000)

        self.vehicle_info_1_msg = self.db.get_message_by_name('Vehicle_Info_1')
        self.vehicle_info_2_msg = self.db.get_message_by_name('Vehicle_Info_2')
        self.info_1_dict = {"APS_Feedback":0, "Break_ACT_Feedback":0,"Gear_Shift_Feedback":5, 
                               "Steering_Angle_Feedback":0, "Switch_State":0}
        self.info_2_dict = {"Override_Feedback":0, "Vehicle_Speed":0, "Turn_Sig_Feed":0,
                               "APS_Feed":0, "BPS_Feed":0}

        self.control_cmd_msg = self.db.get_message_by_name('Control_CMD') 
        self.driving_cmd_msg = self.db.get_message_by_name('Driving_CMD')
        self.control_cmd_dict = {'Override_Off':0,'Alive_Count':0,'Angular_Speed_CMD':30}  
        self.driving_cmd_dict = {'Accel_CMD':650,'Break_CMD':0,'Steering_CMD':0,'Gear_Shift_CMD':5, 'Reserved':0}     

    
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
                if self.vehicle_info_2["Override_Feedback"] != 0:
                    return 0
        except:
            return 0

    def send_control_cmd(self):
        # print("_send_command")
        self.control_cmd_dict['Alive_Count'] += 1
        self.control_cmd_dict['Alive_Count'] %= 256
        #print(self.driving_cmd_info)
        data = self.control_cmd_msg.encode(self.control_cmd_dict)
        message = can.Message(arbitration_id=self.control_cmd_msg.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
    
    def send_driving_cmd(self, accel_cmd, break_cmd, steer_cmd):

        self.driving_cmd_dict["Accel_CMD"] = accel_cmd
        self.driving_cmd_dict["Break_CMD"] = break_cmd
        self.driving_cmd_dict["Steering_CMD"] = steer_cmd

        data = self.driving_cmd_msg.encode(self.driving_cmd_dict)
        message = can.Message(arbitration_id=self.driving_cmd_msg.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
    
    @staticmethod
    def _scaler(old_value, old_min, old_max, new_min, new_max):
        return ((old_value - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min

