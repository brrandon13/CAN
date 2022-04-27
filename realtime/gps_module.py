import geopy
import serial



class GPS:
    def __init__(self):
        try:
            self.gps = serial.Serial('COM1', baudrate=115200)
        except:
            self.gps = 0
    
    def readMsg(self):
        while self.gps:
            msg = self.gps.readline()
            decoded_msg = msg.decode("utf-8")
            data = decoded_msg.split(",")
            if data[0] == '$GPRMC':
                utc = data[1]
                status = data[2]
                if status == 'A':
                    
                    lat = float(data[3])
                    latDir = data[4]
                    lon = float(data[5])
                    lonDir = data[6]
                    kmh = float(data[7]) * 1.852

                    return (lat, latDir, lon, lonDir, kmh)

    def position(self):
        ret = []
        message = self.readMsg()
        if message:
            if message[1] == 'N':
                ret.append(message[0])
            else:
                ret.append(-message[0])
                
            if message[3] == 'E':
                ret.append(message[2])
            else:
                ret.append(-message[2])

        return ret


