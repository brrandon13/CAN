from can_module import CAN 

can = CAN()

i = 0
while i < 100:
    signal = i % 32
    can.info_1_dict["Switch_State"] = signal
    can.send_vehicle_info_1()
    i += 1
    