import cantools
import can
import carla
import math
from agents.navigation.basic_agent import BasicAgent


CONTROL_CMD = 1
DRIVING_CMD = 2

VEHICLE_INFO_1 = 3
VEHICLE_INFO_2 = 4

CYCLE_FPS = 50

class CAN(BasicAgent):
    def __init__(self, vehicle):
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

        BasicAgent.__init__(self,vehicle)
    
    def _get_control(self):
        try:
            msg = self.bus.recv(0.5)  # wait 0.5 sec to get msg then raise error
            data = self.db.decode_message(msg.arbitration_id, msg.data)
            if msg.arbitration_id == self.Control_CMD.frame_id:
                self.control_cmd_info = data
                return CONTROL_CMD
            if msg.arbitration_id == self.Driving_CMD.frame_id:
                self.driving_cmd_info = data
                return DRIVING_CMD
        except:
            return 0
    
    def _send_feedback(self):
        v = self._vehicle.get_velocity()
        c = self._vehicle.get_control()
        self.vehicle_info_1["APS_Feedback"] = round(c.throttle * 3800)
        self.vehicle_info_1["Break_ACT_Feedback"] = round(c.brake * 35000)
        self.vehicle_info_2["Vehicle_Speed"] = min(round(3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)), 255)

        if c.reverse: self.vehicle_info_1["Gear_Shift_Feedback"] = 7
        elif c.gear == 0: self.vehicle_info_1["Gear_Shift_Feedback"] = 6
        else: self.vehicle_info_1["Gear_Shift_Feedback"] = 5

        data = self.Vehicle_Info_1.encode(self.vehicle_info_1)
        message = can.Message(arbitration_id=self.Vehicle_Info_1.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
        data = self.Vehicle_Info_2.encode(self.vehicle_info_2)
        message = can.Message(arbitration_id=self.Vehicle_Info_2.frame_id, data=data, is_extended_id=False)
        self.bus.send(message)

    
    def _get_feedback(self):
        try:
            msg = self.bus.recv(0.5)  # wait 0.5 sec to get msg then raise error
            data = self.db.decode_message(msg.arbitration_id, msg.data)
            if msg.arbitration_id == self.Vehicle_Info_1.frame_id:
                self.vehicle_info_1 = data
                return VEHICLE_INFO_1
            if msg.arbitration_id == self.Vehicle_Info_2.frame_id:
                self.vehicle_info_2 = data
                return VEHICLE_INFO_2
        except:
            return 0

    def _send_command(self):
        self.control_cmd_info['Alive_Count'] += 1
        self.control_cmd_info['Alive_Count'] %= 256

        control = self.run_step()  # calculate from pid control

        accel_cmd = int(self._scaler(control.throttle, 0, 1, 650, 3400))
        break_cmd = int(self._scaler(control.brake, 0, 1, 0, 17000))
        steer_cmd = int(self._scaler(control.steer, -1, 1, -520, 520))

        self.driving_cmd_info["Accel_CMD"] = accel_cmd
        self.driving_cmd_info["Break_CMD"] = break_cmd
        self.driving_cmd_info["Steering_CMD"] = steer_cmd

        data = self.Control_CMD.encode(self.control_cmd_info)
        message = can.Message(arbitration_id=self.Control_CMD.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
        data = self.Driving_CMD.encode(self.driving_cmd_info)
        message = can.Message(arbitration_id=self.Driving_CMD.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
    
    @staticmethod
    def _scaler(old_value, old_min, old_max, new_min, new_max):
        return ((old_value - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min
    

    def run(self):
        self._send_feedback()
        if self._get_feedback():  #  communication established
            self._send_command()  #  calculate control and send to CAN bus

            if self._get_control() == DRIVING_CMD:
                control = carla.VehicleControl()
                control.steer = self._scaler(self.driving_cmd_info["Steering_CMD"],-520,520,-1,1)
                control.throttle = self._scaler(self.driving_cmd_info["Accel_CMD"],650,3400,0,1)
                control.brake = self._scaler(self.driving_cmd_info["Break_CMD"],0,17000,0,1)
                self._vehicle.apply_control()
    