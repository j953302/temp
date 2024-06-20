import serial
import time

port = '/dev/imu'
baud = 115200
ser = serial.Serial(port, baud,timeout=0.5)

mylist = bytes([0xFF,0xAA,0x69,0x88,0xB5])
ser.write(mylist)
time.sleep(0.2)

mylist = bytes([0xFF,0xAA,0x01,0x01,0x00])
ser.write(mylist)
print("Waiting calibration...")
time.sleep(5)

mylist = bytes([0xFF,0xAA,0x00,0x00,0x00])
ser.write(mylist)
print("Done calibration...")
ser.close()