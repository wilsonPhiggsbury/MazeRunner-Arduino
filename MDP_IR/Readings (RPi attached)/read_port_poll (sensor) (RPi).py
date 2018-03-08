import socket
from numpy import median, std
from time import sleep


class TcpClient():
    def __init__(self, ip, port, buffer_size=1024):
        self.ip = ip
        self.port = port
        self.buffer_size = buffer_size
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        self.client_socket.connect((self.ip, self.port))
        print("TcpClient - Connected on {}:{}".format(self.ip, self.port))

    def recv(self):
        try:
            data = self.client_socket.recv(self.buffer_size)
        except:
            return None
        if not data:
            return None
        data_s = data.decode('utf-8')
##        print("TcpClient - Received data: {}".format(data_s))
        return data_s

    def send(self, data):
        try:
            self.client_socket.send(data.encode('utf-8'))
##            print("TcpClient - Sent data: {}".format(data))
        except:
            print("TcpClient - Error sending data: {}".format(data))

    def close_conn(self):
        self.client_socket.close()
        self.connected = False
        print("TcpClient - Disconnected")


client = TcpClient("192.168.7.1", 77)
client.connect()
myfile = open('readvalues_frontsensors.txt','w')
myfile2 = open('readvalues_sidesensors.txt','w')
dist = 0

accumulate = [[],[],[],[],[],[]]
for i in range(6):
    accumulate[i] = list()

while True:
    print('Action: ',end='')
    action = input()
    if action=='':
        client.send('0')
        sleep(1)
        data = client.recv()
        if data is None:
            break
        tmp1 = data.split('\n') # ['1 \t 2 \t 3', '4 \t 5 \t 6']
        tmp2 = tmp1[0].split('\t') + tmp1[1].split('\t') # ['1', '2', '3', '4', '5', '6']
        for i in range(6):
            try:
                accumulate[i].append(float(tmp2[i]))
                print('{:04.2f}'.format(float(tmp2[i])),end='\t')
            except:
                print('Index out of range! The thing you are printing is:\n',data.encode(),'\n')
                exit(0)
        print("\n________________________")
    elif action==' ':
        if len(accumulate[0])==0:
            print('No value to write!')
            continue
        myfile.write('{:04.2f}'.format(float(dist))+'\t')
        myfile2.write('{:04.2f}'.format(float(dist))+'\t')
        dist += 0.25
        print('Values written:')
        for i in range(6):
            writtenData = '{:04.2f}'.format(float(median(accumulate[i])))
            print(writtenData,end='\t')
            if i<3:
                myfile.write(writtenData)
                if i==2:
                    myfile.write('\n')
                    print()
                else:
                    myfile.write('\t')
                
            else:
                myfile2.write(writtenData)
                if i==5:
                    myfile2.write('\n')
                else:
                    myfile2.write('\t')
            accumulate[i] = list()
        myfile.flush()
        myfile2.flush()
        print("\n________________________")
    elif action=='m':
        print("Median:")
        for i in range(6):
            print('{:04.2f}'.format(float(median(accumulate[i]))),end='\t')
        print("\n________________________")
    elif action=='z':
        print('Undo!')
        for i in range(6):
            if len(accumulate[i])>0:
                accumulate[i].pop()
        print("________________________")

    elif action=='s':
        print('Standard deviation:')
        for i in range(6):
            print('{:04.2f}'.format(std(accumulate[i])),end='\t')
        print("\n________________________")
    elif action=='v':
        print('Toggle raw volts...')
        client.send('v')
        sleep(0.5)
        print(client.recv())
        print("\n________________________")
    elif action=='x':
        break
myfile.close()
myfile2.close()
