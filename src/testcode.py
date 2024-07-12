import serial
import time
PORT = '/dev/ttyUSB0'
ser = serial.Serial(PORT, 9600)
data = 1


while(True):
    ser.write(data.encode())
    