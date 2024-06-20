import serial
import time

def nine_to_six(ser):
    ## 解鎖
    mylist = bytes([0xFF,0xAA,0x69,0x88,0xB5])
    ser.write(mylist)
    time.sleep(0.2)

    ## 設置為六軸演算
    mylist = bytes([0xFF,0xAA,0x24,0x01,0x00])
    ser.write(mylist)
    time.sleep(5)

    ## 保存
    mylist = bytes([0xFF,0xAA,0x00,0x00,0x00])
    ser.write(mylist)
    print('Done 9 to 6')

def six_to_nine(ser):
    ## 解鎖
    mylist = bytes([0xFF,0xAA,0x69,0x88,0xB5])
    ser.write(mylist)
    time.sleep(0.2)

    ## 設置為九軸演算
    mylist = bytes([0xFF,0xAA,0x24,0x00,0x00])
    ser.write(mylist)
    time.sleep(5)

    ## 保存
    mylist = bytes([0xFF,0xAA,0x00,0x00,0x00])
    ser.write(mylist)
    print('Done 6 to 9')

port = '/dev/imu'
baud = 115200
ser = serial.Serial(port, baud,timeout=0.5)

if ser.isOpen():
    print("imu connect is open！")
else:
    print("imu connect is not open！")

nine_to_six(ser)

