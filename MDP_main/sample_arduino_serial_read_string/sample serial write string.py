import serial
from time import sleep

port = "COM3"
s = serial.Serial(port)
sleep(2)
while True:
    a = input()
    a += '\n'
    s.write(a[0:2].encode('utf-8'))
    sleep(0.5)
    print('Incoming bytes:',s.in_waiting)
    if s.in_waiting:
        b = s.read(s.in_waiting).decode('utf-8')
        print(b)
