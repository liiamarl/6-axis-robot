from dynamixel_sdk import *
from time import sleep

port1 = PortHandler("/dev/ttyUSB0")
port2 = PortHandler("/dev/ttyUSB1")
packetHandler1 = PacketHandler(1.0)
packetHandler2 = PacketHandler(2.0)

def setup_ports():
    global port1, port2, packetHandler1, packetHandler2
    if port1.openPort():
        print("Succeeded to open port1")
    else:
        print("Failed to open port1")
        quit()
    
    if port2.openPort():
        print("Succeeded to open port2")
    else:
        print("Failed to open port2")
        quit()
    
    if port1.setBaudRate(1000000):
        print("Succeeded to change the baudrate for port1")
    else:
        print("Failed to change the baudrate for port1")
        quit()
    
    if port2.setBaudRate(4000000):
        print("Succeeded to change the baudrate for port2")
    else:
        print("Failed to change the baudrate for port2")
        quit()
        
def enable_torque():
    global port1, port2, packetHandler1, packetHandler2
    
    dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(port1, 1, 24, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler1.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 1 has been successfully connected")
    
    dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(port1, 2, 64, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 2 has been successfully connected")

    dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(port1, 3, 24, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler1.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 3 has been successfully connected")
    
    dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(port2, 4, 64, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 4 has been successfully connected")
    
    dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(port2, 5, 64, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 5 has been successfully connected")
    
    dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(port2, 6, 64, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 6 has been successfully connected")
        
def disable_torque():
    global port1, port2, packetHandler1, packetHandler2
    
    dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(port1, 1, 24, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler1.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 1 has been successfully deconnected")
    
    dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(port1, 2, 64, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 2 has been successfully deconnected")

    dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(port1, 3, 24, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler1.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 3 has been successfully deconnected")
    
    dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(port2, 4, 64, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 4 has been successfully deconnected")
    
    dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(port2, 5, 64, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 5 has been successfully deconnected")
    
    dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(port2, 6, 64, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 6 has been successfully deconnected")
        
        
def move_servo(ID, position):
    global port1, port2, packetHandler1, packetHandler2
    if ID == 1 or ID == 3:
        dxl_comm_result, dxl_error = packetHandler1.write2ByteTxRx(port1, ID, 30, position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler1.getRxPacketError(dxl_error))
    else:
        if ID == 2:
            dxl_comm_result, dxl_error = packetHandler2.write4ByteTxRx(port1, ID, 116, position)
        if ID > 3:
            dxl_comm_result, dxl_error = packetHandler2.write4ByteTxRx(port2, ID, 116, position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler2.getRxPacketError(dxl_error))

def get_servo_position(ID):
    global port1, port2, packetHandler1, packetHandler2
    position = -1
    if ID == 1 or ID == 3:
        position, dxl_comm_result, dxl_error = packetHandler1.read2ByteTxRx(port1, ID, 36)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler1.getRxPacketError(dxl_error))
    else:
        if ID == 2:
            position, dxl_comm_result, dxl_error = packetHandler2.read4ByteTxRx(port1, ID, 132)
        if ID > 3:
            position, dxl_comm_result, dxl_error = packetHandler2.read4ByteTxRx(port2, ID, 132)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler2.getRxPacketError(dxl_error))
    return position

def write_data(ID, adress, lenth, data):
    global port1, port2, packetHandler1, packetHandler2
    if lenth == 1:
        if ID == 1 or ID == 3:
            dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(port1, ID, adress, data)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler1.getRxPacketError(dxl_error))
        else:
            if ID == 2:
                dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(port1, ID, adress, data)
            if ID > 3:
                dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(port2, ID, adress, data)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler2.getRxPacketError(dxl_error))
    if lenth == 2:
        if ID == 1 or ID == 3:
            dxl_comm_result, dxl_error = packetHandler1.write2ByteTxRx(port1, ID, adress, data)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler1.getRxPacketError(dxl_error))
        else:
            if ID == 2:
                dxl_comm_result, dxl_error = packetHandler2.write2ByteTxRx(port1, ID, adress, data)
            if ID > 3:
                dxl_comm_result, dxl_error = packetHandler2.write2ByteTxRx(port2, ID, adress, data)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler2.getRxPacketError(dxl_error))
    if lenth == 4:
        if ID == 1 or ID == 3:
            dxl_comm_result, dxl_error = packetHandler1.write4ByteTxRx(port1, ID, adress, data)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler1.getRxPacketError(dxl_error))
        else:
            if ID == 2:
                dxl_comm_result, dxl_error = packetHandler2.write4ByteTxRx(port1, ID, adress, data)
            if ID > 3:
                dxl_comm_result, dxl_error = packetHandler2.write4ByteTxRx(port2, ID, adress, data)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler2.getRxPacketError(dxl_error))
                
def read_data(ID, adress, lenth):
    global port1, port2, packetHandler1, packetHandler2
    if lenth == 1:
        if ID == 1 or ID == 3:
            data, dxl_comm_result, dxl_error = packetHandler1.read1ByteTxRx(port1, ID, adress)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler1.getRxPacketError(dxl_error))
        else:
            if ID == 2:
                data, dxl_comm_result, dxl_error = packetHandler2.read1ByteTxRx(port1, ID, adress)
            if ID > 3:
                data, dxl_comm_result, dxl_error = packetHandler2.read1ByteTxRx(port2, ID, adress)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler2.getRxPacketError(dxl_error))
    if lenth == 2:
        if ID == 1 or ID == 3:
            data, dxl_comm_result, dxl_error = packetHandler1.read2ByteTxRx(port1, ID, adress)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler1.getRxPacketError(dxl_error))
        else:
            if ID == 2:
                data, dxl_comm_result, dxl_error = packetHandler2.read2ByteTxRx(port1, ID, adress)
            if ID > 3:
                data, dxl_comm_result, dxl_error = packetHandler2.read2ByteTxRx(port2, ID, adress)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler2.getRxPacketError(dxl_error))
    if lenth == 4:
        if ID == 1 or ID == 3:
            data, dxl_comm_result, dxl_error = packetHandler1.read4ByteTxRx(port1, ID, adress)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler1.getRxPacketError(dxl_error))
        else:
            if ID == 2:
                data, dxl_comm_result, dxl_error = packetHandler2.read4ByteTxRx(port1, ID, adress)
            if ID > 3:
                data, dxl_comm_result, dxl_error = packetHandler2.read4ByteTxRx(port2, ID, adress)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler2.getRxPacketError(dxl_error))
    return data

def setup_regul_parameters():
    write_data(1, 26, 1, 1) #CW compliance margin
    write_data(1, 27, 1, 1) #CCW
    write_data(1, 28, 1, 64) #CW compliance slope
    write_data(1, 29, 1, 64) #CCW
    write_data(1, 48, 2, 1) #punch
    
    write_data(2, 84, 2, 2000) #P
    write_data(2, 82, 2, 100) #I
    write_data(2, 80, 2, 15000) #D
    
    write_data(3, 26, 1, 1) #CW compliance margin
    write_data(3, 27, 1, 1) #CCW
    write_data(3, 28, 1, 128) #CW compliance slope
    write_data(3, 29, 1, 32) #CCW
    write_data(3, 48, 2, 1) #punch
    
    write_data(4, 84, 2, 1500)
    write_data(4, 82, 2, 30)
    write_data(4, 80, 2, 5000)
    
    write_data(5, 84, 2, 1000)
    write_data(5, 82, 2, 30)
    write_data(5, 80, 2, 1000)
    
    write_data(6, 84, 2, 1500)
    write_data(6, 82, 2, 30)
    write_data(6, 80, 2, 5000)
    
def setup_speed(ID, speed):
    #speed in degrees per second (weird but ok)
    if ID == 1 or ID == 3:
        write_data(ID, 32, 2, int(speed / 0.666))
    else:
        write_data(ID, 112, 4, int(speed / 1.374))
    
    

