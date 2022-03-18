import can
import cantools
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pygame
import datetime as dt


CAN_FEEDBACK_NAME = ["APS_Feedback", "Break_ACT_Feedback","Steering_Angle_Feedback","Override_Feedback", "Vehicle_Speed","Turn_Sig_Feed"]

class CAN(object):
    def __init__(self):
        # CAN
        self.db = cantools.database.load_file('SantaFe.dbc')
        self.bus = can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=500000)
        self.vehicle_info_1 = self.db.get_message_by_name(
            'Vehicle_Info_1')  # APS_Feedback, Break_ACT_Feedback, Steering_Angle_Feedback
        self.vehicle_info_2 = self.db.get_message_by_name(
            'Vehicle_Info_2')  # Override_Feedback, Vehicle_Speed, Turn_Sig_Feed
        self.Control_CMD = self.db.get_message_by_name('Control_CMD')
        # Override_Off, Alive_Count, Angular_Speed_CMD
        self.Driving_CMD = self.db.get_message_by_name('Driving_CMD')
        # Accel_CMD, Break_CMD, Steering_CMD, Gear_Shift_CMD

        self.timestamp = 0

        # Control CMD
        self.override_off = True
        self.alive_count = 0
        self.angular_speed = 50

        # Driving CMD
        self.Accel_CMD = 0
        self.Break_CMD = 0
        self.Steering_CMD = 0
        self.Gear_Shift_CMD = 6  # N

        self.feedback_info = {"APS_Feedback":0, "Break_ACT_Feedback":0,"Gear_Shift_Feedback":6,
                              "Steering_Angle_Feedback":0,"Override_Feedback":0, "Vehicle_Speed":0,"Turn_Sig_Feed":0}

    def feedback(self):
        msg = 1
        is_updated = False
        while msg is not None:
            msg = self.bus.recv(1)
            data = self.db.decode_message(msg.arbitration_id, msg.data)
            self.timestamp = msg.timestamp

            for feedback_name in CAN_FEEDBACK_NAME:
                if feedback_name in data:
                    self.feedback_info[feedback_name] = data[feedback_name]

            yield self.timestamp, self.feedback_info['Vehicle_Speed'], self.feedback_info['APS_Feedback'], self.feedback_info['Break_ACT_Feedback']

    def recieve_command(self):
        pass


rx_can = CAN()

fig, (ax1, ax2, ax3) = plt.subplots(3,1, sharex=True)  # ax1 : vehicle speed ax2 : APS_Feedback(accel) ax3 : Break_ACT_Feedback(break)

xs, y1, y2, y3 = [], [], [], []

ax1.set_ylim([0, 255])
ax2.set_ylim([0, 3800])
ax3.set_ylim([0, 35000])

def animate(i, xs, y1, y2, y3):
    t, v, a, b = next(rx_can.feedback())
    xs.append(t)
    y1.append(v)
    y2.append(a)
    y3.append(b)

    xs = xs[-1000:]
    y1 = y1[-1000:]
    y2 = y2[-1000:]
    y3 = y3[-1000:]

    ax1.clear()
    ax1.plot(xs, y1)
    ax2.clear()
    ax2.plot(xs, y2)
    ax3.clear()
    ax3.plot(xs,y3)



ani = FuncAnimation(fig, animate, fargs=(xs,y1,y2,y3), interval=20)
plt.show()



