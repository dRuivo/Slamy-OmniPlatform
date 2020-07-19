import serial

ser = serial.Serial('/dev/ttyS1', 19200, timeout=1)
x = ser.read()          # read one byte
s = ser.read(10)        # read up to ten bytes (timeout)
ser.write(b'hello')     # write a string
line = ser.readline()   # read a '\n' terminated line

ser.close()