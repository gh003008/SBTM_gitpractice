from control import *
import time
import socket

########################################################################################################################
# 차랑과의 시리얼통신 open
print("Serial 통신을 시도합니다.")
car = control(port_num='/COM5')
car.open_serial()
print("Serial 통신을 성공!")
########################################################################################################################

########################################################################################################################
i = 0

speed = 50
steer = -100

while True:
    start = time.time()

    i += 1

    if (speed < 0) and (i % 100 == 0):
        speed += 1
        
    car.send_data(speed, steer)
    loop_time = time.time() - start