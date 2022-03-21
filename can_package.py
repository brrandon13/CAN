import cantools
import can

CONTROL_CMD = 1
DRIVING_CMD = 2
CYCLE_FPS = 50

class CAN(object):
    def __init__(self):
        # CAN
        self.db = cantools.database.load_file('SantaFe.dbc')
        self.bus = can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=500000)

        self.Vehicle_Info_1 = self.db.get_message_by_name('Vehicle_Info_1')
        self.Vehicle_Info_2 = self.db.get_message_by_name('Vehicle_Info_2')
        self.vehicle_info_1 = {"APS_Feedback":0, "Break_ACT_Feedback":0,"Gear_Shift_Feedback":5, 
                               "Steering_Angle_Feedback":0, "Switch_State":0}
        self.vehicle_info_2 = {"Override_Feedback":0, "Vehicle_Speed":0, "Turn_Sig_Feed":0,
                               "APS_Feed":0, "BPS_Feed":0}

        self.Control_CMD = self.db.get_message_by_name('Control_CMD') 
        self.Driving_CMD = self.db.get_message_by_name('Driving_CMD')
        self.control_cmd_info = {'Override_Off':0,'Alive_Count':0,'Angular_Speed_CMD':30}  
        self.driving_cmd_info = {'Accel_CMD':650,'Break_CMD':0,'Steering_CMD':0,'Gear_Shift_CMD':5, 'Reserved':0}     