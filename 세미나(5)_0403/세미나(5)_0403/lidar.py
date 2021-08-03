import socket

from math import cos, sin, pi, sqrt, pow, atan2, degrees
import numpy as np
from numpy import unravel_index
import cv2
import scipy.cluster.hierarchy as hcluster
import pickle

class lidar():
    def __init__(self, degree_min, degree_max, range_max, thresh, plot_flag):

        self.HOST = '192.168.0.1'  # TCP/IP 주소
        self.PORT = 2111  # 포트 번호

        self.instruction = 'sRN LMDscandata'
        self.message = chr(2) + self.instruction + chr(3)

        self.background_image = np.zeros((600, 1200), np.uint8)
        self.background_image = cv2.cvtColor(self.background_image, cv2.COLOR_GRAY2BGR)
        for i in range(1, 7):  # 반원 그리기
            cv2.circle(self.background_image, (600, 0), i * 100, (0, 255, 0), 1)

        self.bubble_image = np.zeros((300, 600), np.uint8)
        self.path_image = np.zeros((300, 600), np.uint8)
        self.fusion_image = np.zeros((300, 600), np.uint8)

        self.cluster_data_X = []  # 클러스터 된 데이터의 중심점 저장
        self.cluster_data_Y = []

        self.c_point_x = []
        self.c_point_y = []

        self.degree_min = degree_min  # 최소 측정 각도
        self.degree_max = degree_max  # 최대 측정 각도
        self.range_max = range_max  # 최대 측정 거리
        self.thresh = thresh
        self.plot_flag = plot_flag

    def scan_data(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.HOST, self.PORT))
                s.send(str.encode(self.message))

                self.data = b''
                recv_data = b''
                while recv_data != b'\x03':
                    recv_data = s.recv(1)                    
                    self.data += recv_data
                    

            
            self.raw_data = self.data.split()
            
            
            
        except:
            print("라이다가 응답하지 않습니다.")

    def coordinate_transform(self):
        for i in range(1, 100):
            if self.raw_data[i] == b'9C4' and self.raw_data[i + 1] == b'439':
                real_data = self.raw_data[i + 2: i + 1081 + 2]

                n_min = self.degree_min * 4 + 540
                n_max = self.degree_max * 4 + 540

                real_data = real_data[n_min: n_max + 1]
                real_data = [int(x, 16) for x in real_data]

                Radian = [j / 4 for j in range(self.degree_min * 4, self.degree_max * 4 + 1, 1)]

                for t in range(n_max - n_min, -1, -1):
                    if real_data[t] > self.range_max or real_data[t] == 0:
                        del real_data[t]
                        del Radian[t]

                self.R_max = self.range_max
                self.data_size = len(real_data)

                self.x_data = [int(real_data[r] * sin((Radian[r] / 180) * pi)) for r in range(0, self.data_size)]
                self.y_data = [int(real_data[r] * cos((Radian[r] / 180) * pi)) for r in range(0, self.data_size)]

                self.x_data = np.array(self.x_data)
                self.y_data = np.array(self.y_data)

                self.x_data = self.x_data.reshape(self.data_size, 1)
                self.y_data = self.y_data.reshape(self.data_size, 1)

                self.point = np.hstack([self.x_data, self.y_data])

                break

    def bubble_map(self):
        bubble_map_range_buffer = []
        bubble_map_degrees_buffer = []
        if self.point.size is not 0:
            for i in range(0, len(self.c_point_x)):
                for j in range(0, len(self.c_point_x[i])):
                    x = self.c_point_x[i][j]
                    y = self.c_point_y[i][j]
                    self.bubble_image = cv2.circle(self.bubble_image,
                                                   (int(x * (300 / self.R_max) + 300), int(y * (300 / self.R_max))),
                                                   int(1000 * (300 / self.R_max)), (100, 100, 100), -1)


            for angle in range(-20, 25, 1):
                path = np.zeros((300, 600), np.uint8)
                self.path_image = cv2.line(path, (300, 0), (int(300 * sin((angle / 180) * pi) + 300),
                                                                       int(300 * cos((angle / 180) * pi))), (100, 100, 100), 1)

                self.fusion_image = self.bubble_image + self.path_image

                if 200 in self.fusion_image:
                    y, x = np.where(self.fusion_image == 200)[0][0], np.where(self.fusion_image == 200)[1][0]
                    #print(sqrt(pow(x - 600, 2) + pow(y, 2)))
                    bubble_map_range_buffer.append(sqrt(pow(x - 300, 2) + pow(y, 2)))
                    bubble_map_degrees_buffer.append(angle)
                else:
                    bubble_map_range_buffer.append(300)
                    bubble_map_degrees_buffer.append(angle)

                #print(bubble_map_range_buffer)

            path_range_list = [i for i, j in enumerate(bubble_map_range_buffer) if j == max(bubble_map_range_buffer)]
            path_degrees_list = [bubble_map_degrees_buffer[i] for i in path_range_list]
            #print(path_degrees_list)
            Optimal_angle = min(path_degrees_list, key=lambda x: abs(0 - x))
            self.bubble_map_reset()

            return Optimal_angle

        else:
            Optimal_angle = 0

            return Optimal_angle

    def bubble_map_reset(self):
        self.bubble_image = np.zeros((300, 600), np.uint8)
        self.path_image = np.zeros((300, 600), np.uint8)

    def clustering(self):
        if self.point.size > 5:
            self.c_point_x = []
            self.c_point_y = []

            clusters = hcluster.fclusterdata(self.point, self.thresh, criterion="distance")
            c_max = clusters.max()
            c_min = 1
            clusters = clusters.reshape(self.data_size, 1)

            for i in range(c_min, c_max + 1):
                self.c_X = self.x_data[clusters == i]
                self.c_Y = self.y_data[clusters == i]
                
                ##print(type(self.c_X))    

                self.c_point_x.append(self.c_X)
                self.c_point_y.append(self.c_Y)

                if len(self.c_X) and len(self.c_Y) > 2:
                    
                    self.avg_X = np.mean(self.c_X)
                    self.avg_Y = np.mean(self.c_Y)

                    self.cluster_data_X.append(self.avg_X)
                    self.cluster_data_Y.append(self.avg_Y)
            


    def plotting(self):
        if self.point.size is not 0:
            for i in range(0, len(self.c_point_x)):
                R = np.random.randint(0, 255)
                B = np.random.randint(0, 255)
                G = np.random.randint(0, 255)

                for j in range(0, len(self.c_point_x[i])):
                    x = self.c_point_x[i][j]
                    y = self.c_point_y[i][j]
                    self.background_image = cv2.circle(self.background_image,
                                                       (int(x * (600 / self.R_max) + 600),
                                                        int(y * (600 / self.R_max))), 1, (B, G, R), -1)

            for i in range(0, len(self.cluster_data_X)):
                self.background_image = cv2.circle(self.background_image,
                                                   (int(self.cluster_data_X[i] * (600 / self.R_max) + 600),
                                                    int(self.cluster_data_Y[i] * (600 / self.R_max))), 3, (0, 0, 255), -1)

        image = cv2.flip(self.background_image, 0)

        for i in range(1, 7):
            text = str(int(self.R_max - self.R_max * (i - 1) / 6))
            cv2.putText(image, text + 'mm', ((6 - i) * 100 + 640, 590), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1)

###
        bubble_map = True  # 버블 맵 출력 설정
        if bubble_map is True:
            self.fusion_image = self.fusion_image / 2
            #self.fusion_image = cv2.cvtColor(self.fusion_image, cv2.COLOR_GRAY2BGR)
            image2 = cv2.flip(self.fusion_image, 0)
            cv2.imshow('Bubble', image2)
###

        cv2.imshow('Lidar', image)
        cv2.waitKey(1)

        self.plotting_reset()

    def plotting_reset(self):
        self.background_image = np.zeros((600, 1200), np.uint8)
        self.background_image = cv2.cvtColor(self.background_image, cv2.COLOR_GRAY2BGR)
        for i in range(1, 7):
            cv2.circle(self.background_image, (600, 0), i * 100, (0, 255, 0), 1)

########################################################################################################################
    def sudden_obstacle(self):  # 동적 장애물 미션
        obstacle_detection = 'pass'

        self.scan_data()
        self.coordinate_transform()
        self.clustering()

        if len(self.cluster_data_X) > 0:
            for i in range(0, len(self.cluster_data_X)):
                if self.cluster_data_X[i] < 500 and self.cluster_data_X[i] > -500:
                    if self.cluster_data_Y[i] < 1500:                        
                        obstacle_detection = 'stop'
                        break

        if self.plot_flag:
            self.plotting()

        self.cluster_data_X = []
        self.cluster_data_Y = []

        return obstacle_detection

    def big_bus_sudden_obstacle(self):  # 동적 장애물 미션
        obstacle_detection = 'pass'
        tmp_range = self.range_max

        self.range_max = 10000

        self.scan_data()
        self.coordinate_transform()
        self.clustering()

        if len(self.cluster_data_X) > 0:
            for i in range(0, len(self.cluster_data_X)):
                if self.cluster_data_X[i] < 1000 and self.cluster_data_X[i] > -1000:
                    if self.cluster_data_Y[i] < 2000:
                        obstacle_detection = 'stop'
                        break

        if self.plot_flag:
            self.plotting()

        self.cluster_data_X = []
        self.cluster_data_Y = []

        self.range_max = tmp_range

        return obstacle_detection

    def fixed_obstacle(self):  # 정적 장애물 미션
        self.scan_data()
        self.coordinate_transform()
        self.clustering()

        STEER = self.bubble_map()
        STEER = int(STEER * 2000 / 40)

        if STEER > 2000:
            STEER = 2000
        elif STEER < -1999:
            STEER = -1999

        if self.plot_flag:
            self.plotting()

        self.cluster_data_X = []
        self.cluster_data_Y = []

        return STEER


if __name__ == "__main__":

    lidar_main = lidar(degree_min=-90, degree_max=90, range_max=2000, thresh=50, plot_flag=True)

    #car = control(port_num='COM4')
    #car.open_serial()
    count = 0
    while True:
        STEER = lidar_main.sudden_obstacle()
        #STEER = lidar_main.fixed_obstacle()
        print(STEER)

        if STEER is 'pass':
            #car.send_data(60, 0, 0, 0)
            pass

        elif STEER is 'stop':
            #car.send_data(0, 0, 100, 0)
            pass

        else:
            #car.send_data(60, STEER, 0, 0)
            pass
