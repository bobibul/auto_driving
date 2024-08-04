import ginyung as fl
import serial


PORT = '/dev/ttyUSB0'

if __name__ == "__main__":
    env_line = fl.libCAMERA(cam_num = 2)
    ser = serial.Serial(PORT, 115200)
    count = 0
    while(True):

        env_line.run()
        data = "o" + str(int(-5 * env_line.cam_steer)) + "\n"
        print(data)
        ser.write(data.encode())