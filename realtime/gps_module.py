from geopy import distance
import rospy
from sensor_msgs.msg import NavSatFix


class GPS:
    def __init__(self):
        self.position = None
        
    def gps_callback(self, msg):
        self.position = (msg.latitude, msg.longitude)

    def distance(self, target):
        return distance.distance(self.position, target) * 0.001



