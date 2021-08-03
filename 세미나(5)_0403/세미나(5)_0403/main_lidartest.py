from lidar import *
import time
import socket 


########################################################################################################################
# 라이다 생성
lidar = lidar(degree_min=-90, degree_max=90, range_max=2000, thresh=50, plot_flag=True)
########################################################################################################################



while True:
    state = lidar.sudden_obstacle()
    print(state)


#lidar.scan_data()

    
    