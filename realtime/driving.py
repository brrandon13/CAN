#!/usr/bin/env python3

from can_module import CAN, DRIVE, PARKING
import pygame
import rospy
from sensor_msgs.msg import NavSatFix

global longitude
global latitude

def gps_callback(data):
    global latitude, longitude
    latitude = data.latitude
    longitude = data.longitude

def gps():
    rospy.init_node('topic_reader')
    rospy.Subscriber('gps_out', NavSatFix, gps_callback)
    rospy.spin()


leftdown = (35.229918, 126.840906)
rightup  = (35.230695, 126.842483)

start = (35.230544, 126.841108)
dest = (25.230071, 126.842450)


dt = 0.02  # sec

def main():
    global latitude, longitude

    can = CAN(dest)  # target_position
    gps() # initiate gps subscriber

    clock = pygame.time.Clock()

    # start override
    can.start_autopilot()

    # change gear P to D
    can.change_gear(DRIVE)

    while not can.reached_destination((latitude, longitude)):

        can.control((latitude, longitude))
        clock.tick_busy_loop(50)  # 1 tick = 20ms
        can.get_feedback()

    else:
        # change gear D to P
        can.change_gear(PARKING)

        # finish override
        can.done()
    return 

if __name__=="__main__":
    main()
    

