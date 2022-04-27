from can_module import CAN, DRIVE, PARKING
import pygame
import threading

leftdown = (35.229918, 126.840906)
rightup  = (35.230695, 126.842483)

start = (35.230544, 126.841108)
dest = (25.230071, 126.842450)


dt = 0.02  # sec

can = CAN(dest)  # target_position

clock = pygame.time.Clock()

# start override
can.start_autopilot()

# change gear P to D
can.change_gear(DRIVE)

while True:
    if can.control():
        break
    can.get_feedback()
    
can.change_gear(PARKING)
can.done()





