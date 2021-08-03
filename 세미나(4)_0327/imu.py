import serial

class IMU:
    def __init__(self, port, brate): # baud rate
        self.port = port
        self.brate = brate

        self.message = ''

        self.seri = serial.Serial(self.port, baudrate=self.brate, timeout=None)

    def get_imu_data(self):
        imu_data = self.seri.readline()
        imu_data = imu_data.decode('utf-8')     # 문자열 형태로 변환
        self.message = imu_data.split(',')

        # 코드 처음 실행시 가끔 메세지 제대로 안들어오는 경우 발생할 때 메세지 다시 받아옴
        while len(self.message) != 3:
            print('Message In Aggain')              
            imu_data = self.seri.readline()
            imu_data = imu_data.decode('utf-8')
            self.message = imu_data.split(',')

    def get_yaw(self):
        self.get_imu_data()
        yaw = float(self.message[2])
        yaw = yaw + 90

        if yaw < 0:
            yaw = yaw + 360

        print(yaw)
        return yaw

    def send_command(self, str_command):            # 커맨드 예시 : '<ver>'
        self.seri.write(str.encode(str_command))
        response = self.seri.readline().decode('utf-8').split(',')[0][:4]
        print(response)
        if len(response) < 4:
            print(self.seri.readline().decode('utf-8').split(',')[0][:4])

if __name__ == "__main__":

    imu = IMU(port='COM6', brate=9600)

    while True:
        yaw = imu.get_yaw()
        print(yaw)



imu_test = IMU(port='COM6', brate=9600)
imu_test.get_yaw