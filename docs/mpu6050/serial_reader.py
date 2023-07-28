import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)

while True:
    b = ser.readline()
    try:
        line = b.decode()
        print(line, end="")
    except:
        pass