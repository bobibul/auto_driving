import ginyung as fl
import serial


PORT = '/dev/ttyUSB0'

if __name__ == "__main__":
    # Exercise Environment Setting
    env_line = fl.libCAMERA(cam_num = 2)
    ser = serial.Serial(PORT, 115200)

    while(True):
        frame1 = env_line.jinhyuk_set()


        env_line.run(frame1)
        data = "o" + str(-5*int(env_line.cam_steer)) + "\n"

        ser.write(data.encode())