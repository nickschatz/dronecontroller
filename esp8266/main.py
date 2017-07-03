import socket
import sys
from machine import UART

uart = UART(0, 9600, timeout=0)

def handle(client):
    while True:
        data = client.recv(5)
        if data == b'':
            break
        uart.write(data)
        # ard_return = uart.read()
        # client.send(ard_return)

if __name__ == '__main__':
    HOST = ""
    PORT = 333

    server = socket.socket()
    server.bind((HOST, PORT))
    server.listen(0)
    # print("Starting drone server")
    while True:
        client, address = server.accept()
        # print("Connection from {}".format(address))
        handle(client)
