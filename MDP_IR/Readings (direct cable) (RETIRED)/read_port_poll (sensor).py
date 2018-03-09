############################################
# Read whatever is available on user input #
############################################
import serial
from time import sleep

myfile = open('readvalues_frontsensors.txt','w')
myfile2 = open('readvalues_sidesensors.txt','w')
def appendToFile(line):
    with open("test.txt", "w") as myfile:
        myfile.write(line)
        myfile.close()
        
#global vaiables
port = "COM3"
printcounter = 0
whichFile = 1

while True:
    
    try:
        ser = serial.Serial(port)
        ser.reset_input_buffer()
        break
    except:
        print('Timeout! Retrying...')
print("________Connected to: " + ser.name+"________")

#this will store the line
line = ""


print('begin!')
while True:
    action = input()
    if action=='x':
        break
    elif action=='':
        ser.write(bytes('x\n','utf-8'))
        sleep(0.5)
        while ser.in_waiting:
            readChar = ser.read().decode('utf-8')
            line += readChar
            if readChar=='\n':
                if whichFile==1:
                    myfile.write(line)
                    myfile.flush()
                    whichFile = 2
                    print("Front sensors value:\t",line,end='')
                else:
                    myfile2.write(line)
                    myfile2.flush()
                    whichFile = 1
                    print("Side sensors value:\t",line,end='')
                line = ""
    elif action=='v':
        ser.write(bytes('v\n','utf-8'))
        sleep(0.5)
        while ser.in_waiting:
            readChar = ser.read().decode('utf-8')
            line += readChar
            if readChar=='\n':
                if whichFile==1:
                    myfile.write(line)
                    myfile.flush()
                    whichFile = 2
                    print("Front sensors value:\t",line,end='')
                else:
                    myfile2.write(line)
                    myfile2.flush()
                    whichFile = 1
                    print("Side sensors value:\t",line,end='')
                line = ""
    elif action=='d':
        ser.write(bytes('D\n','utf-8'))
        sleep(0.5)
        while ser.in_waiting:
            readChar = ser.read().decode('utf-8')
            line += readChar
            if readChar=='\n':
                if whichFile==1:
                    myfile.write(line)
                    myfile.flush()
                    whichFile = 2
                    print("Front sensors value:\t",line,end='')
                else:
                    myfile2.write(line)
                    myfile2.flush()
                    whichFile = 1
                    print("Side sensors value:\t",line,end='')
                line = ""

ser.close()
myfile.close()
myfile2.close()

