import serial
from time import sleep

terminatingString = '\n'

port = "COM3"
s = serial.Serial(port)
b = ""
sleep(1)

while True:
    print("Command string: ",end='')
    a = input()
    if a=='exit':
        exit()
    elif a=='D':
        if terminatingString == 'EOL':
            terminatingString = '\n'
        else:
            terminatingString = 'EOL'
    a += '\n'
    s.write(a.encode('utf-8'))
    
    sleep(1)
    while True:
        print('Collecting input...')
        sleep(0.5)
        
        b += s.read(s.in_waiting).decode('utf-8')
        
        if b.find(terminatingString)!=-1:
            print("___________________________")
            print(b.replace(terminatingString,''))
            b = ""
            break
