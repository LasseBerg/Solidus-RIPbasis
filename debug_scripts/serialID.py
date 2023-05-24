import sys
import serial
import time

PREAMBLE_TTY = '/dev/ttyUSB'

def checkTTY(i, port):

	ser = serial.Serial()
	
	ser.timeout = 2
	ser.baudrate = 115200
	ser.port = port
	ser.dtr = False
	ser.rts = False

	ser.open()
	
	#ser.reset_input_buffer()
	#ser.reset_output_buffer()
	
	time.sleep(2)
	ser.write("i\r".encode("utf-8"))

	time.sleep(0.1)
	data = ser.readline().decode('utf-8').strip("\r\n")
	print(data)

	ser.close()


for i in range(0,4,1):
	port = (PREAMBLE_TTY + str(i))
	
	try:
		checkTTY(i, port)
	except:
		pass

print("Error")