import can
import cantools
import pygame
from can_package import CAN
from agents.navigation.basic_agent import BasicAgent

CONTROL_CMD = 1
DRIVING_CMD = 2
CYCLE_FPS = 50

class Control(CAN, BasicAgent):
    
    def __init__(self):
        CAN.__init__(self)   
        BasicAgent.__init__(self)

    def send_command(self):
        self.control_cmd_info['Alive_Count'] += 1
        self.control_cmd_info['Alive_Count'] %= 256

        data = self.Control_CMD.encode(self.control_cmd_info)
        message = can.Message(arbitration_id=self.Control_CMD.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)
        data = self.Driving_CMD.encode(self.driving_cmd_info)
        message = can.Message(arbitration_id=self.Driving_CMD.frame_id, data=data,is_extended_id=False)
        self.bus.send(message)

    

clock = pygame.time.Clock()

vcan = Control()
while True:
    clock.tick_busy_loop(CYCLE_FPS)
    vcan.send_command()

# vcan send_command
# vcan get_feedback