import socket

 

# serverAddressPort   = ("<broadcast>", 1234)
serverAddressPort   = ("192.168.4.255", 1234)
bufferSize          = 1024

 

# Create a UDP socket at client side
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# UDPClientSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
UDPClientSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
UDPClientSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


 

# Send to server using created UDP socket
msgFromClient       = "Hello UDP Server\n"
bytesToSend         = str.encode(msgFromClient)
UDPClientSocket.sendto(bytesToSend, serverAddressPort)
