import socket

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.bind(('', 1234))

while True:
    content = s.recvfrom(1024)
    print(content)
        

print('closing connection')
client.close()