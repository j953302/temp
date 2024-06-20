"""
2022/12/06版本一
主要使用jetson-nano的示範code做改造
目前可以讀取加速度(G)、角速度(degree/s)、角度(rdegree)
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
import time
from transforms3d.euler import euler2quat as quaternion_from_euler

import serial
import os
from math import pi

class ImuNode(Node):
    def __init__(self,name):
        self.ACCData = [0.0]*8
        self.GYROData = [0.0]*8
        self.AngleData = [0.0]*8
        self.FrameState = 0  # What is the state of the judgment
        self.Bytenum = 0  # Read the number of digits in this paragraph
        self.CheckSum = 0  # Sum check bit

        self.a = [0.0]*3
        self.w = [0.0]*3
        self.Angle = [0.0]*3
        self.gyro = [0.0]*3
        self.acc = [0.0]*3
        super().__init__(name)
        port = '/dev/imu'
        baud = 115200
        self.ser = serial.Serial(port, baud,timeout=0.5)
        if self.ser.isOpen():
            self.get_logger().info("imu connect is open！")
        else:
            self.get_logger().error("imu connect is not open！")
        self.calibration()
        self.pub_ = self.create_publisher(Imu,"imu",10)
        timer_period = 0.01  #每0.1s写一章节话
        self.timer = self.create_timer(timer_period, self.timer_callback)  #启动一个定时装置，每 1 s,调用一次time_callback函数
    def timer_callback(self):
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_link"
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.accel_factor = 9.806 #1G = 9.806 m/s**2
        # 角度標準差
        self.imu_msg.orientation_covariance = [ 
            0.0025, 0.0, 0.0,
            0.0, 0.0025, 0.0,
            0.0, 0.0, 0.0025
        ]
        # 角速度標準差
        self.imu_msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]
        # 加速度標準差
        self.imu_msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]
        datahex = self.ser.read(33)
        self.DueData(datahex)
    def DueData(self,inputdata):  # New core procedures, read the data partition, each read to the corresponding array 
        for data in inputdata:  # Traversal the input data
            if self.FrameState == 0:  # When the state is not determined, enter the following judgment
                if data == 0x55 and self.Bytenum == 0:  # When 0x55 is the first digit, start reading data and increment bytenum
                    self.CheckSum = data
                    self.Bytenum = 1
                    continue
                elif data == 0x51 and self.Bytenum == 1:  # Change the frame if byte is not 0 and 0x51 is identified
                    self.CheckSum += data
                    self.FrameState = 1
                    self.Bytenum = 2
                elif data == 0x52 and self.Bytenum == 1:
                    self.CheckSum += data
                    self.FrameState = 2
                    self.Bytenum = 2
                elif data == 0x53 and self.Bytenum == 1:
                    self.CheckSum += data
                    self.FrameState = 3
                    self.Bytenum = 2
            elif self.FrameState == 1:  # acc
                if self.Bytenum < 10:            # Read 8 data
                    self.ACCData[self.Bytenum-2] = data  # Starting from 0
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):  # verify check bit
                        self.acc = self.get_acc(self.ACCData)
                    
                    self.CheckSum = 0  # Each data is zeroed and a new circular judgment is made
                    self.Bytenum = 0
                    self.FrameState = 0
            elif self.FrameState == 2:  # gyro

                if self.Bytenum < 10:
                    self.GYROData[self.Bytenum-2] = data
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):
                        self.gyro = self.get_gyro(self.GYROData)
                    self.CheckSum = 0
                    self.Bytenum = 0
                    self.FrameState = 0
            elif self.FrameState == 3:  # angle

                if self.Bytenum < 10:
                    self.AngleData[self.Bytenum-2] = data
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):
                        try:
                            self.Angle = self.get_angle(self.AngleData)
                            #result = self.acc+self.gyro+self.Angle
                            roll = self.Angle[0]
                            pitch = self.Angle[1]
                            yaw = self.Angle[2]
                            q = quaternion_from_euler(roll*pi/180, pitch*pi/180, yaw*pi/180) # 角度轉rad
                            self.imu_msg.orientation.x = q[1]
                            self.imu_msg.orientation.y = q[2]
                            self.imu_msg.orientation.z = q[3]
                            self.imu_msg.orientation.w = q[0]
                            
                            self.imu_msg.angular_velocity.x = float(self.gyro[0]*pi/180) # units:rad/sec
                            self.imu_msg.angular_velocity.y = float(self.gyro[1]*pi/180)
                            self.imu_msg.angular_velocity.z = float(self.gyro[2]*pi/180)

                            self.imu_msg.linear_acceleration.x = float(self.acc[0]) * self.accel_factor # units:m/sec^2
                            self.imu_msg.linear_acceleration.y = float(self.acc[1]) * self.accel_factor
                            self.imu_msg.linear_acceleration.z = float(self.acc[2]) * self.accel_factor
                            
                            if self.imu_msg.linear_acceleration.z >11: # 11 is assume number
                                print("z too big error")
                                print(self.imu_msg.linear_acceleration.z)
                            elif self.imu_msg.linear_acceleration.z == 0.0:
                                print("z is zero error")
                                print(self.imu_msg.linear_acceleration.z)
                            else:
                                self.pub_.publish(self.imu_msg)

                            # print("acc:%10.3f %10.3f %10.3f \ngyro:%10.3f %10.3f %10.3f \nangle:%10.3f %10.3f %10.3f" % result)
                            print(yaw)
                        
                            
                        except AttributeError:
                            continue
                    self.CheckSum = 0
                    self.Bytenum = 0
                    self.FrameState = 0


    def get_acc(self,datahex):
        axl = datahex[0]
        axh = datahex[1]
        ayl = datahex[2]
        ayh = datahex[3]
        azl = datahex[4]
        azh = datahex[5]
        k_acc = 16.0
        acc_x = (axh << 8 | axl) / 32768.0 * k_acc
        acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
        acc_z = (azh << 8 | azl) / 32768.0 * k_acc
        if acc_x >= k_acc:
            acc_x -= 2 * k_acc
        if acc_y >= k_acc:
            acc_y -= 2 * k_acc
        if acc_z >= k_acc:
            acc_z -= 2 * k_acc
        return acc_x, acc_y, acc_z


    def get_gyro(self,datahex):
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


    def get_angle(self,datahex):
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

    def calibration(self):
        # 三軸加速度歸零
        mylist = bytes([0xFF,0xAA,0x69,0x88,0xB5])
        self.ser.write(mylist)
        time.sleep(0.2)

        mylist = bytes([0xFF,0xAA,0x01,0x01,0x00])
        self.ser.write(mylist)
        self.get_logger().info('Waiting calibration...')
        time.sleep(5)

        mylist = bytes([0xFF,0xAA,0x00,0x00,0x00])
        self.ser.write(mylist)
        self.get_logger().info('Done calibration...')

def main(args=None):
    """
    ros2运行该节点的入口函数，可配置函数名称
    """    
    print(os.path.abspath(__file__))
    rclpy.init(args=args) # 初始化rclpy
    node = ImuNode("IMU_data")  # 新建一个节点
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown() # rcl关闭
