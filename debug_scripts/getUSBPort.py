import serial
import time
import json

usbs = {
    "SIC": 9999,
    "TEM": 9999,
    "SEN": 9999,
    "CAN": 9999,
    "SER": 9999
}

PREAMBLE_TTY = '/dev/ttyUSB'

def checkTTY(i):

	port = (PREAMBLE_TTY + str(i))

	ser = serial.Serial()
	
	ser.timeout = 2
	ser.baudrate = 115200
	ser.port = port
	ser.open()
	print(f"Open Serial Port: {port}")
	

	time.sleep(2.5)
	ser.write("i\r".encode("utf-8"))

	time.sleep(0.1)
	data = ser.readline().decode('utf-8').strip("\r\n")

	for j in range(len(usbs)):
		key = list(usbs.keys())[j]
		if(data == key):
			usbs[key] = port

	ser.close()


for i in range(0,5,1):
	try:
		checkTTY(i)
	except:
		pass

print(usbs)

# Serializing json
json_object = json.dumps(usbs, indent=4)
 
# Writing to sample.json
with open("/home/robot/debug_scripts/usbs.json", "w") as outfile:
    outfile.write(json_object)

