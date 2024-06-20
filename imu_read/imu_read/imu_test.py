import serial
import time

ACCData = [0.0]*8
GYROData = [0.0]*8
AngleData = [0.0]*8
TimeData = [0.0]*8
FrameState = 0  # What is the state of the judgment
Bytenum = 0  # Read the number of digits in this paragraph
CheckSum = 0  # Sum check bit

a = [0.0]*3
w = [0.0]*3
Angle = [0.0]*3


def DueData(inputdata):  # New core procedures, read the data partition, each read to the corresponding array 
    global FrameState    # Declare global variables
    global Bytenum
    global CheckSum
    global acc
    global gyro
    global Angle
    for data in inputdata:  # Traversal the input data
        
        if FrameState == 0:  # When the state is not determined, enter the following judgment
            if data == 0x55 and Bytenum == 0:  # When 0x55 is the first digit, start reading data and increment bytenum
                #print(data)
                CheckSum = data
                Bytenum = 1
                continue
            elif data == 0x51 and Bytenum == 1:  # Change the frame if byte is not 0 and 0x51 is identified
                CheckSum += data
                FrameState = 1
                Bytenum = 2
            elif data == 0x52 and Bytenum == 1:
                CheckSum += data
                FrameState = 2
                Bytenum = 2
            elif data == 0x53 and Bytenum == 1:
                CheckSum += data
                FrameState = 3
                Bytenum = 2
            elif data == 0x50 and Bytenum == 1:
                print('有時間輸出')
                CheckSum += data
                FrameState = 4
                Bytenum = 2
        elif FrameState == 1:  # acc

            if Bytenum < 10:            # Read 8 data
                ACCData[Bytenum-2] = data  # Starting from 0
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):  # verify check bit
                    acc = get_acc(ACCData)
                CheckSum = 0  # Each data is zeroed and a new circular judgment is made
                Bytenum = 0
                FrameState = 0
        elif FrameState == 2:  # gyro

            if Bytenum < 10:
                GYROData[Bytenum-2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):
                    gyro = get_gyro(GYROData)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0
        elif FrameState == 3:  # angle

            if Bytenum < 10:
                AngleData[Bytenum-2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):
                    Angle = get_angle(AngleData)
                    result = acc+gyro+Angle
                    print("acc:%10.3f %10.3f %10.3f \ngyro:%10.3f %10.3f %10.3f \nangle:%10.3f %10.3f %10.3f" % result)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0
        elif FrameState == 4:  # time
            if Bytenum < 10:
                TimeData[Bytenum-2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):
                    time = get_time(TimeData)
                    print(time)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0


def get_time(datahex):
    yy = datahex[0]
    mm = datahex[1]
    dd = datahex[2]
    hh = datahex[3]
    mn = datahex[4]
    ss = datahex[5]
    ##k_acc = 16.0
    yy = (yy) / 32768.0
    return yy


def get_acc(datahex):
    axl = datahex[0]
    axh = datahex[1]
    ayl = datahex[2]
    ayh = datahex[3]
    azl = datahex[4]
    azh = datahex[5]
    k_acc = 16.0
    acc_x = (axh << 8 | axl) / 32768.0 *k_acc
    acc_y = (ayh << 8 | ayl) / 32768.0 *k_acc
    acc_z = (azh << 8 | azl) / 32768.0 *k_acc
    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z -= 2 * k_acc
    return acc_x, acc_y, acc_z


def get_gyro(datahex):
    wxl = datahex[0]
    wxh = datahex[1]
    wyl = datahex[2]
    wyh = datahex[3]
    wzl = datahex[4]
    wzh = datahex[5]
    k_gyro = 2000.0
    gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
    gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
    gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >= k_gyro:
        gyro_z -= 2 * k_gyro
    return gyro_x, gyro_y, gyro_z


def get_angle(datahex):
    rxl = datahex[0]
    rxh = datahex[1]
    ryl = datahex[2]
    ryh = datahex[3]
    rzl = datahex[4]
    rzh = datahex[5]
    k_angle = 180.0
    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >= k_angle:
        angle_z -= 2 * k_angle
    return angle_x, angle_y, angle_z

def setting():
    packet = bytearray()
    packet.append(0xFF)
    packet.append(0xAA)
    packet.append(0x01)
    packet.append(0x04)
    packet.append(0x00)

    packet_save = bytearray()
    packet_save.append(0xFF)
    packet_save.append(0xAA)
    packet_save.append(0x24)
    packet_save.append(0x01)
    packet_save.append(0x00)

    packet_unlock = bytearray()
    packet_unlock.append(0xFF)
    packet_unlock.append(0xAA)
    packet_unlock.append(0x69)
    packet_unlock.append(0x88)
    packet_unlock.append(0xB5)

    ser.write(packet_unlock)
    time.sleep(1) #要等一下才可以修改
    ser.write(packet)
    time.sleep(1) #要等一下才可以修改
    ser.write(packet_save)
    print("finish setting!")

port = '/dev/imu'
baud = 9600
ser = serial.Serial(port, baud,timeout=0.5)
print("imu serial done")

#setting()
while(1):
    datahex = ser.read(33)
    #print(type(datahex))
    print(datahex)
    #print(datahex.decode('UTF-8'))
    DueData(datahex)
    #break


