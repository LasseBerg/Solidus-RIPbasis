import serial
import time
import sys

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
		command = input("Command: ")
		if(command == "exit"):
			break
		ser.write((command+"\r").encode("utf-8"))
		time.sleep(0.1)
		data = ser.readline().decode('utf-8').strip("\r\n")
		print(data)
	ser.close()



try:
	checkTTY(int(sys.argv[1]))
except:
	pass
