import serial


if __name__ == '__main__':
    serial_port = serial.Serial('/dev/ttyUSB0')

    while True:
        line = serial_port.readline()
        line = line.decode()
        line = line.strip()
        print(line)
