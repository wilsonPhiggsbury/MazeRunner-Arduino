import serial
from time import sleep

terminatingString = 'EOL'

port = "COM3"
s = serial.Serial(port)
b = ""
sleep(1)
print("Debug? y/n: ",end='')
d = input()
    
while True:
    print("Command string: ",end='')
    a = input()
    if a=='exit':
        exit()
    a += '\n'
    s.write(a.encode('utf-8'))
    
    sleep(1)
    if d=='y':
        while True:
            sleep(0.5)
            b += s.read(s.in_waiting).decode('utf-8')
            print('Collecting input...')
            if b.find(terminatingString)!=-1:
                print("___________________________")
                print(b.replace(terminatingString,''))
                b = ""
                break

    else:
        sleep(3)
        b = s.read(s.in_waiting).decode('utf-8')
        print(b)
