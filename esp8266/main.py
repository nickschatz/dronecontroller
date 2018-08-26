import socket
import select
import sys
from machine import UART

uart = UART(0, 230400, timeout=0)

def debug_send(data):
    client.sendall(data)

def handle(client):
    while True:
        # Use select to keep this from blocking and clogging the uart reads
        readable, _, _ = select.select([client], [], [])
        if client in readable:
            data = client.recv(16)
            if len(data) == 0:
                break
            uart.write(data)
        if uart.any() > 0:
            ard_return = uart.read()
            if b'\x03' in ard_return:
                debug_send(b"Got stop command from drone\n")
                raise KeyboardInterrupt
            debug_send(ard_return)

if __name__ == '__main__':
    HOST = ""
    PORT = 333

    server = socket.socket()
    server.bind((HOST, PORT))
    server.listen(0)
    # print("Starting drone server")
    while True:
        client, address = server.accept()
        # debug_send("Connection from {}\n".format(address))
        handle(client)
