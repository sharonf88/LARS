import socket

 

# serverAddressPort   = ("<broadcast>", 1234)
serverAddressPort   = ("192.168.4.255", 1234)
bufferSize          = 1024

 

# Create a UDP socket at client side
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# UDPClientSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
UDPClientSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
UDPClientSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


 
import time
import math
t = 0
amplitude = 0.2
omega = 1/10
while True:
    wave = amplitude * math.sin(t * omega) 
    print(wave)
    time.sleep(0.01)
    t+=1

    # Send to server using created UDP socket
    # msgFromClient       = "Hello UDP Server\n"
    msgFromClient       = "sine"
    bytesToSend         = str.encode(msgFromClient + str(wave))
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)