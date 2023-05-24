import serial
import time

PREAMBLE_TTY = '/dev/ttyUSB'

def checkTTY(i):

	port = (PREAMBLE_TTY + str(i))

	ser = serial.Serial()
	
	ser.timeout = 2
	ser.baudrate = 115200
	ser.port = port
	ser.open()
	

	time.sleep(2.5)

	while True:
		ser.write(("g\r").encode("utf-8"))
		time.sleep(0.1)
		data = ser.readline().decode('utf-8').strip("\r\n")
		print(data.split(" ")[3])
	ser.close()



try:
	checkTTY(2)
except:
	pass
