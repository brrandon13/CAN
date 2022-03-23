from distutils.cygwinccompiler import CygwinCCompiler
from importlib.resources import is_resource
import can
import cantools
import pygame
from can_package import CAN
import random

from can_send_cmd_message import CYCLE_FPS

CONTROL_CMD = 1
DRIVING_CMD = 2


class Car(CAN):
    def __init__(self):
        CAN.__init__(self)

    def get_control(self):
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
            pass
            return 0

    def apply_control(self):
        if self.get_control() == CONTROL_CMD:
            self.vehicle_info_2['Override_Feedback'] = 0
        if self.get_control() == DRIVING_CMD:

            # apply accel
            accel_feed = max(min(3400,random.normalvariate(self.control_cmd_info['Accel_CMD'],3)),650)
            accel_feed = round((accel_feed - 650) * 3800 / (3400-650))
            self.vehicle_info_1['APS_Feedback'] = accel_feed # 650-3400 -> 0-3800
            # apply break
            break_feed = max(min(17000, random.normalvariate(self.driving_cmd_info['Break_CMD'])), 0)  # 0-17000 -> 0-35000
            break_feed = round(break_feed * 17000 / 35000)
            self.vehicle_info_1['Brake_ACT_Feedback'] = break_feed
            # apply gear
            self.vehicle_info_1['Gear_Shift_Feedback'] = self.driving_cmd_info['Gear_Shift_CMD']
            # apply steer
            cur_steer = self.vehicle_info_1['Steering_Angle_Feedback']
            d_steer = (self.driving_cmd_info['Steering_CMD'] - cur_steer)
            if abs(d_steer) > self.control_cmd_info['Angular_Speed_CMD']:
                d_steer =  self.control_cmd_info['Angular_Speed_CMD'] if d_steer > 0 else -self.control_cmd_info['Angular_Speed_CMD']
            cur_steer += d_steer // CYCLE_FPS  # (1/dt = cycle/t)
            self.vehicle_info_1['Steering_Angle_Feedback'] = round(cur_steer)
        else:
            pass
    
    def send_feedback(self):
        data = self.Vehicle_Info_1.encode(self.vehicle_info_1)
        message = can.Message(arbitration_id=self.Vehicle_Info_1.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
        data = self.Vehicle_Info_2.encode(self.vehicle_info_2)
        message = can.Message(arbitration_id=self.Vehicle_Info_2.frame_id, data=data, is_extended_id=False)
        self.bus.send(message)

clock = pygame.time.Clock()

vcan = Car()
while True:
    clock.tick_busy_loop(CYCLE_FPS)
    vcan.apply_control()
    vcan.send_feedback()

# vcan apply_control()
# vcan send_feedback()