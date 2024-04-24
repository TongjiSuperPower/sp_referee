import serial

port = "/dev/ttyUSB0"
baudrate = 115200

ser = serial.Serial(port,baudrate)

while True:
    while ser.read() != b"\xA5":
        pass
    data = ser.read(38)
    for byte in data:
        print(f"{byte:02X}",end=" ")
    print()

ser.close