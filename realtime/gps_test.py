import rospy
import time
from sensor_msgs.msg import NavSatFix


class GPS:
    def __init__(self):
        self.latitude = 0
        self.longitude = 0
        self.init()

    def init(self):
        rospy.init_node('gps_test')
        rospy.Subscriber('/gps/fix', NavSatFix, self.callback)
        rospy.spin()

    def callback(self,data):
        self.latitude = data.latitude
        self.longitude = data.longitude

    def get_current_position(self):
        print(f"Lat: {self.latitude} Lon: {self.longitude}")


gps = GPS()
t = 0
while t < 20:
    gps.get_current_position()
    time.sleep(1)
    t += 1

