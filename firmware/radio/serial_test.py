import serial
import struct
import time 

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=57600, timeout=1) 

def write_read():	
	ser.write(bytearray([20, 10, 10, 10, 0, 0])) 

	msg = ser.read(24)
	return(msg)

while True:
	value = write_read()
	
	try:
		[acc0, acc1, acc2, gyro0, gyro1, gyro2] = struct.unpack('6f', value)
		print(f"{acc0:7.2f} {acc1:7.2f} {acc2:7.2f} {gyro0:7.2f} {gyro1:7.2f} {gyro2:7.2f}", end="\r")
	except:
		pass