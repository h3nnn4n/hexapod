#!/usr/bin/env python3

import serial

from decouple import config


SERIAL_PORT = config("SERIAL_PORT", default="/dev/ttyUSB0")
BAUD_RATE = config("BAUD_RATE", default=115200)
SERIAL_TIMEOUT = config("SERIAL_TIMEOUT", default=1, cast=int)


if __name__ == '__main__':
    serial_port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)

    while True:
        line = serial_port.readline()

        try:
            line = line.decode()
            line = line.strip()
            print(line)
        except UnicodeDecodeError:
            pass
