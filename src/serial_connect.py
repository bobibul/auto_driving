import serial
import time

class SerialConnect():
    def __init__(self, port):
        try:
            self.ser = serial.Serial(port, 9600)
            time.sleep(1)
            print("시리얼 포트 열기 성공")

        except:
            print("시리얼 포트 열기 실패")
            exit()

    def angle_output(self, angle):
        self.ser.write(f'{str(angle).encode()}')

    def speed_output(self, speed):
        self.ser.write(f'{str(speed).encode()}')

