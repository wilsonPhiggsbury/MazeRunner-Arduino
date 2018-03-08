##########################################
# Keep reading until encounter new line #
##########################################
import serial
from time import sleep

myfile = open('readvalues_spam.txt','w')
def appendToFile(line):
    with open("test.txt", "w") as myfile:
        myfile.write(line)
        myfile.close()
        
#global vaiables
port = "COM11"
printcounter = 0

while True:
    
    try:
        ser = serial.Serial(port)
        ser.reset_input_buffer()
        break
    except serial.SerialException as e:
        print('Timeout! Retrying...')
        print(e)
        sleep(1)
print("________Connected to: " + ser.name+"________")

#this will store the line
line = ""
for i in range(3,0,-1):
    print(i,"...",sep='',end='')
    sleep(1)

print('begin!')
ser.write(b'0')
while True:
##    if printcounter > 1000:
##        print("Exhausted")
##        printcounter = 0
##    else:
##        printcounter += 1
    while ser.in_waiting:
        readChar = ser.read().decode('utf-8')
        line += readChar
        if readChar == '\n':
            myfile.write(line)
            print("This line is:",line,end='')
            line = ""
            myfile.flush()

ser.close()
myfile.close()

