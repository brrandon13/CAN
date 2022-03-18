import can
import cantools
import time
import pygame

db = cantools.database.load_file('SantaFe.dbc')
bus = can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=500000)

vehicle_info_1 = db.get_message_by_name('Vehicle_Info_1')
vehicle_info_2 = db.get_message_by_name('Vehicle_Info_2')

def feedback():
    msg = bus.recv()
    data = db.decode_message(msg.arbitration_id, msg.data)
    if msg.arbitration_id == vehicle_info_1.frame_id:
        print('sdgf')
    else:
        print('not')


clock = pygame.time.Clock()
while True:
    clock.tick_busy_loop(5)
    feedback()