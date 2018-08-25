#!/usr/bin/python

import serial
import sys

wport = serial.Serial("/dev/ttyUSB0", 230400)
wport.timeout = None

while True:
    try:
        datum = int.from_bytes(wport.read(1), byteorder="big", signed=True)
        print(datum, end=" ")
        if datum == -128:
            print("")
    except KeyboardInterrupt as e:
        wport.write(b'\x03')
        raise e