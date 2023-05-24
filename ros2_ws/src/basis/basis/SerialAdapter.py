# SerialAdapter
#	Modul for Sending and Reciving Serial Data drom Arduinos
# 
# V1.1
# 
# 20.4.2023
# Benjamin Zbinden

import serial
import time
import json

def writeReadSerial(ser, msg):
	ser.write(msg.encode('utf-8'))
	return readSerial(ser)

def writeSerial(ser, msg):
  ser.write(msg.encode('utf-8'))

def readSerial(ser):
	c = ''
	value = ''

	while c != '\n':
		try:
			c = ser.read(1).decode('utf-8')
			if c == '':
				print("Error: serial timeout")

			value += c
		except:
			value += " "
	value = value.strip('\r\n')
	ser.reset_input_buffer()
	return value


def loadUSB(name):
	print('Load Port Config.')
	# Opening JSON file
	with open('/home/robot/debug_scripts/usbs.json', 'r') as openfile:
		# Reading from json file
		usbs = json.load(openfile)
		port = usbs[name]
		if(port == 9999):
			print("Device not Connectet !!!")
			exit()

		return port

def openSerial(port):
	ser = serial.Serial()
	ser.timeout = 2
	ser.baudrate = 115200
	ser.port = port
	ser.open()
	ser.reset_input_buffer()
	ser.reset_output_buffer()
	print('Opening Serial Port: ' + port)
	time.sleep(2.5)
	print('Serial Ready')
	return ser
