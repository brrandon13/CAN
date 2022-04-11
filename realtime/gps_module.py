import geopy
import serial

ser = 0

class GPS:
    def __init__(self):
        try:
            self.gps = serial.Serial('COM1', baudrate=115200)
        except:
            self.gps = 0
    
    def readMsg(self):
        while True:
            msg = self.gps.readline()
            decoded_msg = msg.decode("utf-8")
            data = decoded_msg.split(",")
            if data[0] == '$GPRMC':
                utc = data[1]
                status = data[2]
                if status == 'A':
                    
                    lat = data[3]
                    latDir = data[4]
                    lon = data[5]
                    lonDir = data[6]
                    kmh = data[7] * 1.852

                    return [lat, latDir, lon, lonDir, kmh]
        return []



