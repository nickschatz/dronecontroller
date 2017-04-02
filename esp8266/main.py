import socket
from machine import SPI

baud = 115200
spi = SPI(1, baudrate=baud, polarity=0, phase=0)
spi.init(baudrate=baud)

def twos(i):
    i = i & 0xFF
    if i & 0b10000000 == 0b10000000:
        return i - (1<<8)
    return i

def handle(client):
    while True:
        x = twos(client.recv(1)[0])
        y = twos(client.recv(1)[0])
        z = twos(client.recv(1)[0])
        s = twos(client.recv(1)[0])
        print("X: {} Y: {} Z: {} S: {}".format(x, y, z, s))

if __name__ == '__main__':
    HOST = ""
    PORT = 333

    server = socket.socket()
    server.bind((HOST, PORT))
    server.listen(0)
    print("Starting drone server")
    spi.write(b"Die Ente ready")
    while True:
        client, address = server.accept()
        print("Connection from {}".format(address))
        # spi.write(bytes("Connection from {}:{}".format(address)))
        handle(client)
