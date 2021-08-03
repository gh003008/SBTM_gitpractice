import serial
import socket
import time

import signal # install pyserial module

class control():
    def __init__(self, port_num):
        self.RMID = 184 # MDUI
        self.TMID = 172 # PC
        self.ID = 1 # 제어기의 ID
        self.port_num = port_num

        self.sign_number = ''
        self.traffic_light = ''

        self.recv_num_list = list()
        self.recv_light_list = list()

        self.yolo_accuracy = 0.2
        self.buffer_length = 5

    def open_serial(self):
        self.ser = serial.Serial(
            port=self.port_num,
            baudrate=57600,
        )

    def free_car(self):
        self.DATA = bytearray(12)

        self.DATA[0] = self.RMID
        self.DATA[1] = self.TMID
        self.DATA[2] = self.ID
        self.DATA[3] = 252 # PID_ROBOT_CMD
        self.DATA[4] = 6
        self.DATA[5] = 0 #D1
        self.DATA[6] = 0
        self.DATA[7] = 0
        self.DATA[8] = 0
        self.DATA[9] = 0
        self.DATA[10] = 0 #D6
        CHK = (~(sum(self.DATA[0:-1]) & 255) + 1) & 255
        self.DATA[11] = CHK

        self.ser.write(bytes(self.DATA))

    def send_data(self, SPEED, ANGLE):
        SPEED = int(SPEED)
        ANGLE = int(ANGLE)

        self.DATA = bytearray(12)

        self.DATA[0] = self.RMID
        self.DATA[1] = self.TMID
        self.DATA[2] = self.ID
        self.DATA[3] = 252 # PID_ROBOT_CMD
        self.DATA[4] = 6 # 6 Bytes to send(D1~D6)

        self.DATA[5] = 1 #D1

        if SPEED >= 0:
            self.DATA[6] = SPEED % 256
            self.DATA[7] = SPEED // 256
        else:
            SPEED = -SPEED
            self.DATA[6] = 255 - SPEED % 256
            self.DATA[7] = 255 - SPEED // 256

        if ANGLE >= 0:
            self.DATA[8] = ANGLE % 256
            self.DATA[9] = ANGLE // 256
        else:
            ANGLE = -ANGLE
            self.DATA[8] = 255 - ANGLE % 256
            self.DATA[9] = 255 - ANGLE // 256

        self.DATA[10] = 0 #D6
        CHK = (~(sum(self.DATA[0:-1]) & 255) + 1) & 255
        self.DATA[11] = CHK

        #self.ser.write(bytes(self.DATA))
        self.ser.write(self.DATA)
