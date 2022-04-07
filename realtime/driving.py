from can_module import CAN
import pygame
import threading


DRIVE =  5
PARKING = 0

dt = 0.02  # sec

can = CAN(80)  # target_distance

clock = pygame.time.Clock()

# start override
# run this step until override_feedback is on & APM, ASM, AGM is ready
while can.info_1_dict["Override_Feedback"] != 1 or not can.is_car_ready(): 
   clock.tick_busy_loop(50)
   th1 = threading.Thread(target=can.send_control_cmd)
   th2 = threading.Thread(target=can.get_vehicle_info_1)
   th1.join()
   th2.join()

# change gear P to D
can.change_gear(DRIVE)

while True:
    can.control()
    th1 = threading.Thread(target=can.send_control_cmd)
    th2 = threading.Thread(target=can.send_driving_cmd)

    th1.join()
    th2.join()

    th3 = threading.Thread(target=can.get_vehicle_info_1)
    th4 = threading.Thread(target=can.get_vehicle_info_2)

    if can.cur_dist >= can.target_dist:
        break

can.change_gear(PARKING)
can.done()

th1 = threading.Thread(target=can.send_control_cmd)
th2 = threading.Thread(target=can.send_driving_cmd)

th1.join()
th2.join()




