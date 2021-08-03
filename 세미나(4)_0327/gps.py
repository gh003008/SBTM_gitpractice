# author : hydragon516

import cv2
from imu import *
import numpy as np
import keyboard
from math import cos, sin, radians, sqrt, pow, degrees, acos
import re
import time


class gps():
    def __init__(self, port, brate, gps_filename, gps_filename_ex, gps_filename_park, mode, degree_mode, start_range_num, plot_flag):
        self.port = port  # GPS 시리얼 포트 번호
        self.brate = brate
        self.gps_filename = gps_filename  # 메인 GPS 맵 txt 파일
        self.gps_filename_ex = gps_filename_ex  # 차선 변경 GPS 맵 txt 파일
        self.gps_filename_park = gps_filename_park  # 주차 GPS 맵 txt 파일
        self.park_num = int(re.findall("\d+", gps_filename_park)[0])  # 주차 GPS 파일에서 주차 번호 추출
        self.mode = mode  # 모드는 test, record, trace
        self.degree_mode = degree_mode  # imu는 IMU로, gps는 GPS로 각도 정보를 받아옴
        self.plot_flag = plot_flag  # plot 모드 설정

        self.limit_range = 1.5  # waypoint 통과기준 거리 설정
        self.parameter_m = 0.00000899321  # GPS 거리 환산 인자
        self.Measure = 1  # 척도 변경

        self.GNGGA_message = []
        self.GNRMC_message = []

        self.Lat_buffer = []
        self.Long_buffer = []
        self.Position_state_buffer = None
        self.Satellites_used_buffer = None
        self.Speed_buffer = None
        self.Degrees_buffer = None
        self.IMU_Degrees_buffer = None

        # 주차 타겟 위치 설정
        if self.park_num == 1:
            self.parking_target = [37.23932933333333, 126.77328566666665]
        elif self.park_num == 2:
            self.parking_target = [37.2393525, 126.77330500000001]
        elif self.park_num == 3:
            self.parking_target = [37.23937633333333, 126.77332316666667]
        elif self.park_num == 4:
            self.parking_target = [37.23939966666667, 126.77333750000001]
        elif self.park_num == 5:
            self.parking_target = [37.23942216666667, 126.773351]
        elif self.park_num == 6:
            self.parking_target = [37.23944616666667, 126.77337033333333]

        self.buffer_state = 'not yet'
        self.nearest_point_state = True
        self.parking_state = 'forward'

        self.target_range = 100

        self.range_num = 1
        self.record_mode = False
        self.record_num = 0
        self.Lat_record_buffer = 0
        self.Long_record_buffer = 0

        self.key_press = False

        self.gps_load_state = False
        self.speed_state = False

        self.next_waypoint_index = 0
        self.next_waypoint_index_ex = 0
        self.next_waypoint_index_park_f = 0
        self.next_waypoint_index_park_b = 0

        self.waypoint_degrees = 0
        self.difference_angle = 0
        self.STEER_angle = 0

        self.Record_Lat = []
        self.Record_Long = []
        self.Record_range_num = []
        self.Record_Lat_ex = []
        self.Record_Long_ex = []
        self.Record_range_num_ex = []
        self.Record_Lat_park_f = []
        self.Record_Long_park_f = []
        self.Record_range_num_park_f = []
        self.Record_Lat_park_b = []
        self.Record_Long_park_b = []
        self.Record_range_num_park_b = []

        self.start_range_num = start_range_num

        self.waypoint_length = []
        self.waypoint_length_ex = []
        self.waypoint_length_park_f = []
        self.waypoint_length_park_b = []
        self.out_of_length_num = []
        self.out_of_length_num_ex = []
        self.out_of_length_num_park_f = []
        self.out_of_length_num_park_b = []

        self.lane_change_mode = False  # 차선 변경 모드
        self.parking_mode = False  # 주차 모드

        # 이미지 초기화
        self.background_image = np.zeros((1000, 1000), np.uint8)
        self.background_image = cv2.cvtColor(self.background_image, cv2.COLOR_GRAY2BGR)
        for i in range(0, 10):
            self.background_image = cv2.circle(self.background_image, (500, 500), int(500 - 50 * i), (0, 0, 255), 1)
        self.background_image = cv2.circle(self.background_image, (500, 500),
                                           int(self.limit_range * 50), (255, 255, 0), 1)

        # GPS 파일 읽기
        if self.mode is 'record':
            self.f = open(self.gps_filename, 'w')
        if self.mode is 'trace':
            self.f = open(self.gps_filename, 'r')
            self.f_ex = open(self.gps_filename_ex, 'r')
            self.f_park = open(self.gps_filename_park, 'r')

        # 시리얼 통신 설정
        self.seri = serial.Serial(self.port, baudrate=self.brate, timeout=0)
        self.seri.set_buffer_size(rx_size=10000, tx_size=1024)

        if self.degree_mode is 'imu':
            self.imu = IMU(port='COM10', brate=115200)

        self.gngga_regexp = re.compile(r"[$][G][N][G][G][A]([\s\S]*?)[\\][n]")
        self.gnrmc_regexp = re.compile(r"[$][G][N][R][M][C]([\s\S]*?)[\\][n]")

    # GPS 데이터 파싱, GPS 버퍼의 상태가 'ready'일때 'ready'를 반환, 아니면 'not yet'을 반환
    def get_gps_data(self):
        gps_data = self.seri.read(10000)
        self.seri.flushInput()      # 수신 데이터 캐시 폐기
        gps_data = str(gps_data)

        try:
            self.GNGGA_message = self.gngga_regexp.findall(gps_data)[-1].split(",")
            self.GNGGA_message[0] = "GNGGA"
        except:
            pass
        try:
            self.GNRMC_message = self.gnrmc_regexp.findall(gps_data)[-1].split(",")
            self.GNRMC_message[0] = "GNRMC"
        except:
            pass

        if self.degree_mode is 'imu':
            self.IMU_Degrees_buffer = self.imu.get_yaw()

        if len(self.GNGGA_message) > 0 and len(self.GNRMC_message) > 0:
            self.buffer_state = self.gps_buffer()

        elif len(self.GNGGA_message) == 0 or len(self.GNRMC_message) == 0:
            self.buffer_state = 'not yet'

        if self.buffer_state is 'ready':
            return 'ready'

        elif self.buffer_state is 'not yet':
            return 'not yet'

    # GPS 버퍼, 필요한 모든 데이터가 저장되면 'ready'를 반환, 아니면 'not yet'을 반환
    def gps_buffer(self):
        if (self.GNGGA_message[2] is not '') and (self.GNGGA_message[4] is not ''):
            if len(self.Lat_buffer) > 0 and len(self.Long_buffer) > 0:
                # 위도 경도의 데이터 형식을 dd.dddd로 변경
                Lat = float(self.GNGGA_message[2])
                Lat = ((Lat % 100) / 60) + ((Lat - Lat % 100) / 100)
                Long = float(self.GNGGA_message[4])
                Long = ((Long % 100) / 60) + ((Long - Long % 100) / 100)

                if Lat != self.Lat_buffer[0] and Long != self.Long_buffer[0]:
                    self.Lat_buffer.insert(0, Lat)
                    self.Long_buffer.insert(0, Long)

            elif len(self.Lat_buffer) == 0 and len(self.Long_buffer) == 0:
                # 위도 경도의 데이터 형식을 dd.dddd로 변경
                Lat = float(self.GNGGA_message[2])
                Lat = ((Lat % 100) / 60) + ((Lat - Lat % 100) / 100)
                Long = float(self.GNGGA_message[4])
                Long = ((Long % 100) / 60) + ((Long - Long % 100) / 100)

                self.Lat_buffer.insert(0, Lat)
                self.Long_buffer.insert(0, Long)

        if len(self.Lat_buffer) > 500 and len(self.Long_buffer) > 500:
            # 지금까지 지나온 점 중 500개만 저장
            self.Lat_buffer = self.Lat_buffer[0: 500]
            self.Long_buffer = self.Long_buffer[0: 500]

        if self.GNGGA_message[6] is not '':
            self.Position_state_buffer = int(self.GNGGA_message[6])

        if self.GNGGA_message[7] is not '':
            self.Satellites_used_buffer = int(self.GNGGA_message[7])

        if self.GNRMC_message[7] is not '':
            speed_raw = float(self.GNRMC_message[7])
            speed_raw = speed_raw * 1.852  # 속도의 단위를 km/h로 환산
            self.Speed_buffer = speed_raw

        if self.GNRMC_message[8] is not '':
            Degrees = float(self.GNRMC_message[8])
            self.Degrees_buffer = Degrees

        if self.mode is 'record' or 'test':
            if len(self.Lat_buffer) > 0 and len(self.Long_buffer) > 0 and (self.Position_state_buffer is not None) and (
                    self.Satellites_used_buffer is not None) and (self.Speed_buffer is not None):
                state = 'ready'
                return state
            else:
                state = 'not yet'
                return state

        if self.mode is 'trace':
            if len(self.Lat_buffer) > 0 and len(self.Long_buffer) > 0 and (self.Position_state_buffer is not None) and (
                    self.Satellites_used_buffer is not None) and (self.Speed_buffer is not None) and (
                    self.Degrees_buffer is not None):
                state = 'ready'
                return state
            else:
                state = 'not yet'
                return state

    # record 모드에서 GPS 데이터 저장, 저장 형태는 [미션 번호, 위도, 경도]
    def recorder(self):
        if (self.Lat_record_buffer is not self.Lat_buffer[0]) and (self.Long_record_buffer is not self.Lat_buffer[0]):
            data = str(self.range_num) + ',' + str(self.Lat_buffer[0]) + ',' + str(self.Long_buffer[0])
            self.Lat_record_buffer = self.Lat_buffer[0]
            self.Long_record_buffer = self.Long_buffer[0]
            data = "%s\n" % data
            self.record_num = self.record_num + 1
            self.f.write(data)

    # trace 모드에서 GPS 파일을 읽어 버퍼에 저장
    def load_data(self):
        # 메인 GPS 데이터 읽기
        while True:
            line = self.f.readline()
            data = line.strip().split(',')

            if not line:
                break

            if int(data[0]) >= self.start_range_num:
                self.Record_range_num.append(int(data[0]))
                self.Record_Lat.append(float(data[1]))
                self.Record_Long.append(float(data[2]))

        self.f.close()

        print('저장된 데이터 로드 완료')

        # 차선 변경 GPS 데이터 읽기
        while True:
            line = self.f_ex.readline()
            data = line.strip().split(',')

            if not line:
                break

            self.Record_range_num_ex.append(int(data[0]))
            self.Record_Lat_ex.append(float(data[1]))
            self.Record_Long_ex.append(float(data[2]))

        self.f_ex.close()

        print('저장된 데이터 로드 완료')

        # 주차 GPS 데이터 읽기
        while True:
            line = self.f_park.readline()
            data = line.strip().split(',')

            if not line:
                break

            self.Record_range_num_park_f.append(int(data[0]))
            self.Record_Lat_park_f.append(float(data[1]))
            self.Record_Long_park_f.append(float(data[2]))

        self.Record_range_num_park_b = self.Record_range_num_park_f.copy()
        self.Record_Lat_park_b = self.Record_Lat_park_f.copy()
        self.Record_Long_park_b = self.Record_Long_park_f.copy()

        self.Record_range_num_park_b.reverse()
        self.Record_Lat_park_b.reverse()
        self.Record_Long_park_b.reverse()

        self.f_park.close()

        print('저장된 데이터 로드 완료')

    # RTK 문제로 GPS 절대 좌표가 이동한 경우 시작 점을 기준으로 모든 저장 값을 이동
    # 평상시에는 사용할 필요 없음
    def gps_shifter(self):
        start_Latitude, start_Longitude = self.Lat_buffer[0], self.Long_buffer[0]
        shift_Lat = self.Record_Lat[0] - start_Latitude
        shift_Long = self.Record_Long[0] - start_Longitude

        for i in range(0, len(self.Record_Lat)):
            self.Record_Lat[i] = self.Record_Lat[i] - shift_Lat
            self.Record_Long[i] = self.Record_Long[i] - shift_Long

        for i in range(0, len(self.Record_Lat_ex)):
            self.Record_Lat_ex[i] = self.Record_Lat_ex[i] - shift_Lat
            self.Record_Long_ex[i] = self.Record_Long_ex[i] - shift_Long

        print('gps 보정 완료')

    # 주석을 제거하면 gps 데이터 보정
    def gps_loader(self):
        self.load_data()
        #self.gps_shifter()

    # 차선 변경 모드 선택
    def lane_change(self, state):
        if state is True:
            self.lane_change_mode = True

        elif state is False:
            self.lane_change_mode = False

    # 주차 모드 선택
    def park(self, state):
        if state is True:
            self.parking_mode = True

        elif state is False:
            self.parking_mode = False

    # 주차 상태 변경, 'forward'는 주차장 진입, 'backward'는 주차장 이탈
    def park_sate(self, state):
        if state is 'forward':
            self.parking_state = 'forward'

        elif state is 'backward':
            self.parking_state = 'backward'

    # 현재 위치와 주차 타겟 지점까지의 거리 반환
    def park_distance(self):
        center_point_y = self.Lat_buffer[0]
        center_point_x = self.Long_buffer[0]

        length = sqrt((pow(center_point_y - self.parking_target[0], 2)
                       + (pow(center_point_x - self.parking_target[1], 2)))) / self.parameter_m

        return length

    # 현재 위치(그리고 현재 미션번호)에서 가장 가까운 waypoint 탐색
    # mode가 Ture이면 가장 가까운 점에서 10개 이후의 waypoint 탐색
    def get_nearest_point(self, mode):
        waypoint_length = list()
        waypoint_length_ex = list()
        start_Latitude, start_Longitude = self.Lat_buffer[0], self.Long_buffer[0]
        if self.lane_change_mode is False:
            for i in range(0, len(self.Record_Lat)):

                if int(self.Record_range_num[i]) is not int(self.Record_range_num[0]):
                    break

                length = sqrt((pow(start_Latitude - self.Record_Lat[i], 2) + (
                    pow(start_Longitude - self.Record_Long[i], 2)))) / self.parameter_m
                waypoint_length.append(length)
            nearest_index = waypoint_length.index(min(waypoint_length))
            if mode:
                nearest_index = nearest_index + 10
                try:
                    self.Record_range_num = self.Record_range_num[nearest_index:]
                    self.Record_Long = self.Record_Long[nearest_index:]
                    self.Record_Lat = self.Record_Lat[nearest_index:]
                except:
                    nearest_index = nearest_index - 10
                    self.Record_range_num = self.Record_range_num[nearest_index:]
                    self.Record_Long = self.Record_Long[nearest_index:]
                    self.Record_Lat = self.Record_Lat[nearest_index:]
            else:
                self.Record_range_num = self.Record_range_num[nearest_index:]
                self.Record_Long = self.Record_Long[nearest_index:]
                self.Record_Lat = self.Record_Lat[nearest_index:]

            print('가장 가까운 점 찾기 완료')

        elif self.lane_change_mode is True:
            for i in range(0, len(self.Record_Lat_ex)):
                length = sqrt((pow(start_Latitude - self.Record_Lat_ex[i], 2) + (
                    pow(start_Longitude - self.Record_Long_ex[i], 2)))) / self.parameter_m
                waypoint_length_ex.append(length)
            nearest_index = waypoint_length_ex.index(min(waypoint_length_ex))
            if mode:
                nearest_index = nearest_index + 10
                try:
                    self.Record_range_num_ex = self.Record_range_num_ex[nearest_index:]
                    self.Record_Long_ex = self.Record_Long_ex[nearest_index:]
                    self.Record_Lat_ex = self.Record_Lat_ex[nearest_index:]
                except:
                    nearest_index = nearest_index - 10
                    self.Record_range_num_ex = self.Record_range_num_ex[nearest_index:]
                    self.Record_Long_ex = self.Record_Long_ex[nearest_index:]
                    self.Record_Lat_ex = self.Record_Lat_ex[nearest_index:]
                print('가장 가까운 점 찾기 완료')
            else:
                self.Record_range_num_ex = self.Record_range_num_ex[nearest_index:]
                self.Record_Long_ex = self.Record_Long_ex[nearest_index:]
                self.Record_Lat_ex = self.Record_Lat_ex[nearest_index:]

    # 진행할 waypoint의 각도와 현재 차량의 각도 차이를 계산, -180 ~ 180의 값을 반환
    def get_angle(self, unit1, unit2):
        phi = abs(unit2 - unit1) % 360
        sign = 1
        if not ((unit1 - unit2 >= 0 and unit1 - unit2 <= 180) or (unit1 - unit2 <= -180 and unit1 - unit2 >= -360)):
            sign = -1

        if phi > 180:
            result = 360 - phi

        else:
            result = phi

        return result * sign


    # 항법주행 함수, 반환값은 STEER
    def tracing(self):
        if self.nearest_point_state == True:
            self.get_nearest_point(False)
            self.nearest_point_state = False

        if self.lane_change_mode is False:  # 차선 변경 전
            if len(self.Record_Long) > 2:
                if len(self.Record_Long) > 80:
                    for i in range(0, 80):
                        center_point_y = self.Lat_buffer[0]
                        center_point_x = self.Long_buffer[0]

                        length = sqrt((pow(center_point_y - self.Record_Lat[i], 2) + (
                            pow(center_point_x - self.Record_Long[i], 2)))) / self.parameter_m

                        if length < self.limit_range:
                            self.out_of_length_num.append(i)
                else:
                    for i in range(0, len(self.Record_Long)):
                        center_point_y = self.Lat_buffer[0]
                        center_point_x = self.Long_buffer[0]

                        length = sqrt((pow(center_point_y - self.Record_Lat[i], 2) + (
                            pow(center_point_x - self.Record_Long[i], 2)))) / self.parameter_m

                        if length < self.limit_range:
                            self.out_of_length_num.append(i)

            elif len(self.Record_Long) <= 2:
                return 'finish'

            next_waypoint_index = 0

            if len(self.out_of_length_num) > 0:
                next_waypoint_index = max(self.out_of_length_num) + 1
                self.out_of_length_num = []

            self.Record_Lat = self.Record_Lat[next_waypoint_index:]
            self.Record_Long = self.Record_Long[next_waypoint_index:]
            self.Record_range_num = self.Record_range_num[next_waypoint_index:]

            x_c = (self.Record_Long[0] - self.Long_buffer[0]) / self.parameter_m
            y_c = (self.Record_Lat[0] - self.Lat_buffer[0]) / self.parameter_m

            x_L_c = int(x_c * 50 / self.Measure)
            y_L_c = int(y_c * 50 / self.Measure)

            L = sqrt((pow(x_L_c, 2) + (pow(y_L_c, 2))))

            self.waypoint_degrees = 180 - degrees(acos((- y_L_c) / L))

            if x_L_c < 0:
                self.waypoint_degrees = - self.waypoint_degrees + 360

            if self.Degrees_buffer is None:
                self.Degrees_buffer = 0

            if self.degree_mode is 'imu':
                self.difference_angle = self.get_angle(self.waypoint_degrees, self.IMU_Degrees_buffer)
            elif self.degree_mode is 'gps':
                self.difference_angle = self.get_angle(self.waypoint_degrees, self.Degrees_buffer)

            self.STEER_angle = int(self.difference_angle * 2000 / 60)

            if self.STEER_angle > 2000:
                self.STEER_angle = 2000

            elif self.STEER_angle < -1999:
                self.STEER_angle = -1999

            return self.STEER_angle

        elif self.lane_change_mode is True:  # 차선변경
            if len(self.Record_Long_ex) > 2:
                if len(self.Record_Long_ex) > 80:
                    for i in range(0, 80):
                        center_point_y = self.Lat_buffer[0]
                        center_point_x = self.Long_buffer[0]

                        length = sqrt((pow(center_point_y - self.Record_Lat_ex[i], 2) + (
                            pow(center_point_x - self.Record_Long_ex[i], 2)))) / self.parameter_m
                        print('ok')
                        if length < self.limit_range:
                            self.out_of_length_num_ex.append(i)
                else:
                    for i in range(0, len(self.Record_Long_ex)):
                        center_point_y = self.Lat_buffer[0]
                        center_point_x = self.Long_buffer[0]

                        length = sqrt((pow(center_point_y - self.Record_Lat_ex[i], 2) + (
                            pow(center_point_x - self.Record_Long_ex[i], 2)))) / self.parameter_m

                        if length < self.limit_range:
                            self.out_of_length_num_ex.append(i)

            elif len(self.Record_Long_ex) <= 2:
                return 'finish'

            next_waypoint_index_ex = 0

            if len(self.out_of_length_num_ex) > 0:
                next_waypoint_index_ex = max(self.out_of_length_num_ex) + 1
                self.out_of_length_num_ex = []

            self.Record_Lat_ex = self.Record_Lat_ex[next_waypoint_index_ex:]
            self.Record_Long_ex = self.Record_Long_ex[next_waypoint_index_ex:]
            self.Record_range_num_ex = self.Record_range_num_ex[next_waypoint_index_ex:]

            x_c = (self.Record_Long_ex[0] - self.Long_buffer[0]) / self.parameter_m
            y_c = (self.Record_Lat_ex[0] - self.Lat_buffer[0]) / self.parameter_m

            x_L_c = int(x_c * 50 / self.Measure)
            y_L_c = int(y_c * 50 / self.Measure)

            L = sqrt((pow(x_L_c, 2) + (pow(y_L_c, 2))))

            self.waypoint_degrees = 180 - degrees(acos((- y_L_c) / L))

            if x_L_c < 0:
                self.waypoint_degrees = - self.waypoint_degrees + 360

            if self.Degrees_buffer is None:
                self.Degrees_buffer = 0

            if self.degree_mode is 'imu':
                self.difference_angle = self.get_angle(self.waypoint_degrees, self.IMU_Degrees_buffer)
            elif self.degree_mode is 'gps':
                self.difference_angle = self.get_angle(self.waypoint_degrees, self.Degrees_buffer)

            self.STEER_angle = int(self.difference_angle * 2000 / 60)

            if self.STEER_angle > 2000:
                self.STEER_angle = 2000

            elif self.STEER_angle < -1999:
                self.STEER_angle = -1999

            return self.STEER_angle

    # 이미지 출력 함수
    def plotting(self):
        # 현재까지 지나온 점 표시
        if self.Degrees_buffer is None:
            self.Degrees_buffer = 0

        for i in range(1, len(self.Lat_buffer)):
            center_point_y = self.Lat_buffer[0]
            center_point_x = self.Long_buffer[0]
            self.background_image = cv2.circle(self.background_image, (500, 500), 3, (0, 255, 0), -1)

            x = (self.Long_buffer[i] - center_point_x) / self.parameter_m
            y = (self.Lat_buffer[i] - center_point_y) / self.parameter_m

            x_L = int(x * 50 / self.Measure)
            y_L = int(y * 50 / self.Measure)

            self.background_image = cv2.circle(self.background_image, (500 + x_L, 500 - y_L), 3, (0, 255, 0), -1)

        # 고정상태, 위성개수, 속도, 진행 각도 표시
        if self.Position_state_buffer is 0:
            cv2.putText(self.background_image, 'State : Void', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255),
                        2)

        elif self.Position_state_buffer is 1:
            cv2.putText(self.background_image, 'State : Active', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255),
                        2)

        cv2.putText(self.background_image, 'Satellites : ' + str(self.Satellites_used_buffer), (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(self.background_image, 'Speed : ' + "%0.2f" % self.Speed_buffer + 'km/h', (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        if self.degree_mode is 'imu':
            cv2.putText(self.background_image, 'Degrees : ' + "%0.1f" % self.IMU_Degrees_buffer, (10, 115),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        elif self.degree_mode is 'gps':
            cv2.putText(self.background_image, 'Degrees : ' + "%0.1f" % self.Degrees_buffer, (10, 115),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # 진행 방향 표시
        if self.degree_mode is 'imu':
            radian_degrees_data = radians(self.IMU_Degrees_buffer)
            self.background_image = cv2.circle(self.background_image, (900, 85), 70, (255, 255, 255), 1)
            self.background_image = cv2.line(self.background_image, (900, 85),
                                             (int(900 + 70 * sin(radian_degrees_data)),
                                              int(85 - 70 * cos(radian_degrees_data))), (255, 0, 0), 2)

        elif self.degree_mode is 'gps':
            radian_degrees_data = radians(self.Degrees_buffer)
            self.background_image = cv2.circle(self.background_image, (900, 85), 70, (255, 255, 255), 1)
            self.background_image = cv2.line(self.background_image, (900, 85),
                                             (int(900 + 70 * sin(radian_degrees_data)),
                                              int(85 - 70 * cos(radian_degrees_data))), (255, 0, 0), 2)

        # record 모드일때 record 상태 표시
        if self.mode is 'record':
            cv2.putText(self.background_image, 'Record state : ' + str(self.record_mode), (10, 165),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(self.background_image, 'Record num : ' + str(self.record_num), (10, 205),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(self.background_image, 'Current range num : ' + str(self.range_num), (10, 305),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # trace 모드일때 waypoint 표시
        if self.mode is 'trace':
            for i in range(0, len(self.Record_Long)):
                center_point_y = self.Lat_buffer[0]
                center_point_x = self.Long_buffer[0]

                x = (self.Record_Long[i] - center_point_x) / self.parameter_m
                y = (self.Record_Lat[i] - center_point_y) / self.parameter_m

                x_L_R = int(x * 50 / self.Measure)
                y_L_R = int(y * 50 / self.Measure)

                self.background_image = cv2.circle(self.background_image, (500 + x_L_R, 500 - y_L_R), 3,
                                                   (255, 0, 255), -1)
                cv2.putText(self.background_image, str(self.Record_range_num[i]), (490 + x_L_R, 590 - y_L_R),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            for i in range(0, len(self.Record_Long_ex)):
                center_point_y = self.Lat_buffer[0]
                center_point_x = self.Long_buffer[0]

                x = (self.Record_Long_ex[i] - center_point_x) / self.parameter_m
                y = (self.Record_Lat_ex[i] - center_point_y) / self.parameter_m

                x_L_R = int(x * 50 / self.Measure)
                y_L_R = int(y * 50 / self.Measure)

                self.background_image = cv2.circle(self.background_image, (500 + x_L_R, 500 - y_L_R), 3,
                                                   (0, 255, 255), -1)
                cv2.putText(self.background_image, 'ex', (490 + x_L_R, 590 - y_L_R),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            if self.parking_state == 'forward':
                for i in range(0, len(self.Record_Long_park_f)):
                    center_point_y = self.Lat_buffer[0]
                    center_point_x = self.Long_buffer[0]

                    x = (self.Record_Long_park_f[i] - center_point_x) / self.parameter_m
                    y = (self.Record_Lat_park_f[i] - center_point_y) / self.parameter_m

                    x_L_R = int(x * 50 / self.Measure)
                    y_L_R = int(y * 50 / self.Measure)

                    self.background_image = cv2.circle(self.background_image, (500 + x_L_R, 500 - y_L_R), 3,
                                                       (0, 255, 255), -1)
                    cv2.putText(self.background_image, 'park', (490 + x_L_R, 590 - y_L_R),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            elif self.parking_state == 'backward':
                for i in range(0, len(self.Record_Long_park_b)):
                    center_point_y = self.Lat_buffer[0]
                    center_point_x = self.Long_buffer[0]

                    x = (self.Record_Long_park_b[i] - center_point_x) / self.parameter_m
                    y = (self.Record_Lat_park_b[i] - center_point_y) / self.parameter_m

                    x_L_R = int(x * 50 / self.Measure)
                    y_L_R = int(y * 50 / self.Measure)

                    self.background_image = cv2.circle(self.background_image, (500 + x_L_R, 500 - y_L_R), 3,
                                                       (0, 255, 255), -1)
                    cv2.putText(self.background_image, 'park', (490 + x_L_R, 590 - y_L_R),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.putText(self.background_image, 'Remaining waypoints : ' + str(len(self.Record_Long)), (10, 165),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            cv2.putText(self.background_image, 'Waypoint degrees : ' + "%0.1f" % self.waypoint_degrees, (10, 205),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(self.background_image, 'Difference degrees : ' + "%0.1f" % self.difference_angle, (10, 235),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(self.background_image, 'STEER : ' + "%0.1f" % self.STEER_angle, (10, 265),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            if self.parking_mode is False:
                if self.lane_change_mode is True:
                    cv2.putText(self.background_image, 'Current range num : ' + 'ex', (10, 305),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                    x_c = (self.Record_Long_ex[0] - self.Long_buffer[0]) / self.parameter_m
                    y_c = (self.Record_Lat_ex[0] - self.Lat_buffer[0]) / self.parameter_m

                    x_L_c = int(x_c * 50 / self.Measure)
                    y_L_c = int(y_c * 50 / self.Measure)

                    self.background_image = cv2.line(self.background_image, (500, 500), (500 + x_L_c, 500 - y_L_c), (0, 255, 0), 2)

                elif self.lane_change_mode is False:
                    cv2.putText(self.background_image, 'Current range num : ' + str(self.Record_range_num[0]), (10, 305),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                    x_c = (self.Record_Long[0] - self.Long_buffer[0]) / self.parameter_m
                    y_c = (self.Record_Lat[0] - self.Lat_buffer[0]) / self.parameter_m

                    x_L_c = int(x_c * 50 / self.Measure)
                    y_L_c = int(y_c * 50 / self.Measure)

                    self.background_image = cv2.line(self.background_image, (500, 500), (500 + x_L_c, 500 - y_L_c),
                                                     (0, 255, 0), 2)

            elif self.parking_mode is True:
                if self.parking_state == 'forward':
                    cv2.putText(self.background_image, 'Current range num : ' + 'forward', (10, 305),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                    x_c = (self.Record_Long_park_f[0] - self.Long_buffer[0]) / self.parameter_m
                    y_c = (self.Record_Lat_park_f[0] - self.Lat_buffer[0]) / self.parameter_m

                    x_L_c = int(x_c * 50 / self.Measure)
                    y_L_c = int(y_c * 50 / self.Measure)

                    self.background_image = cv2.line(self.background_image, (500, 500), (500 + x_L_c, 500 - y_L_c),
                                                     (0, 255, 0), 2)

                elif self.parking_state == 'backward':
                    cv2.putText(self.background_image, 'Current range num : ' + 'backward', (10, 305),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                    x_c = (self.Record_Long_park_f[0] - self.Long_buffer[0]) / self.parameter_m
                    y_c = (self.Record_Lat_park_f[0] - self.Lat_buffer[0]) / self.parameter_m

                    x_L_c = int(x_c * 50 / self.Measure)
                    y_L_c = int(y_c * 50 / self.Measure)

                    self.background_image = cv2.line(self.background_image, (500, 500), (500 + x_L_c, 500 - y_L_c),
                                                     (0, 255, 0), 2)

        cv2.imshow('GPS', self.background_image)

        cv2.waitKey(1)

        self.plotting_reset()

    # 출력 이미지 초기화
    def plotting_reset(self):
        self.background_image = np.zeros((1000, 1000), np.uint8)
        self.background_image = cv2.cvtColor(self.background_image, cv2.COLOR_GRAY2BGR)
        for i in range(0, 10):
            self.background_image = cv2.circle(self.background_image, (500, 500), int(500 - 50 * i), (0, 0, 255), 1)
        self.background_image = cv2.circle(self.background_image, (500, 500), 75, (255, 255, 0), 1)


########################################################################################################################
#                                           미션에 사용되는 주요 함수                                                  #
########################################################################################################################

    def gps_test(self):
        data_state = self.get_gps_data()

        if keyboard.is_pressed('q'):
            return 'finish'

        if data_state is 'ready':
            if self.plot_flag:
                self.plotting()

        return 'pass'

    def gps_recorder(self):
        data_state = self.get_gps_data()

        if data_state is 'ready':
            if keyboard.is_pressed('r'):
                self.record_mode = True

            elif keyboard.is_pressed('s'):
                self.record_mode = False

            elif keyboard.is_pressed('q'):
                self.f.close()
                return self.range_num, 'finish'

            if self.key_press is False:
                if keyboard.is_pressed('c'):
                    self.range_num = self.range_num + 1
                    self.key_press = True

            elif self.key_press is True:
                if keyboard.is_pressed('c') is False:
                    self.key_press = False

            if self.record_mode is True:
                self.recorder()

            if self.plot_flag:
                self.plotting()

            return self.range_num, 'pass'

        else:
            return self.range_num, 'pass'

    def gps_tracer(self):
        data_state = self.get_gps_data()

        if keyboard.is_pressed('q'):
            return self.Record_range_num[0], 'finish'

        if data_state is 'not yet':
            return 'none', 'pass'

        elif data_state is 'ready' and self.gps_load_state is False:
            self.gps_loader()
            self.gps_load_state = True
            return 'none', 'pass'

        elif data_state is 'ready' and self.gps_load_state is True:
            if self.plot_flag:
                self.plotting()

            if self.parking_mode is False:
                STEER = self.tracing()

                return self.Record_range_num[0], STEER

            elif self.parking_mode is True:
                STEER = self.parking()

                return self.Record_range_num[0], STEER

        else:
            return 'none', 'pass'
