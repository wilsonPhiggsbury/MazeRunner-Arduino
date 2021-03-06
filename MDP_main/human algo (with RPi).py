import socket
import time

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

##print("Debug: ")
b = 'n'
if(b=='y'):

    debug = True
else:
    debug = False
if debug:
    eol = '\n\n'
else:
    eol = '\n'

b=''
while True:
    a = str(input())
    if a=='':
        a = 'a'
    if a=='D':
        debug = not debug
        if debug:
            eol = '\n\n'
        else:
            eol = '\n'
    client.send(a)
    if debug:
        
        while True:
            print('Collecting input...')
            sleep(0.5)
            b += client.recv()
            print("State",state)
            if b.find(eol)!=-1 or state==1:
                state = 0
                s.cancel(eventobj)
                s.run()
                print("___________________________")
                print(b.replace(eol,''))
                b = ""
                break
    else:
            time.sleep(1)
            b = client.recv()
            if b!='\n':
                print(b)
            else:
                print('_____')
