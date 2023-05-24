# TempPrintBridge
#	Send Controll commands to Temperature Print
# and get periodicly data from Temperature Print
# 
# V1.0
# 
# 31.3.2023
# Benjamin Zbinden

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

#Custom imports
import basis.SerialAdapter as seradapt


class TempPrintBridge(Node):

	def __init__(self, port):
		super().__init__('TempPrintBridge')

		self.currentCommand = ""
		self.lastCommand = ""

		self.ser = seradapt.openSerial(port)

		seradapt.writeReadSerial(self.ser, "m 1\r")
		self.temp_tempBattery_publisher 		=   self.create_publisher(String, 'temp_tempBattery', 10)
		self.temp_tempDCDC_publisher  			=   self.create_publisher(String, 'temp_tempDCDC', 10)
		self.temp_tempOutside_publisher 		=   self.create_publisher(String, 'temp_tempOutside', 10)
		self.temp_humidityOutside_publisher =   self.create_publisher(String, 'temp_humidityOutside', 10)

		################################### SENSOR COMMAND LISTENER ################################################
		self.temp_command_subscription = self.create_subscription(
			String,
			'temp_command',
			self.temp_command_listener,
			10)
		self.temp_command_subscription  # prevent unused variable warning


		################################### TIMERS FOR CALLING SENSOR BOARD ###########################################
		timer_period_temp_data = float(100)/1000  # seconds
		self.get_temp_data_timer = self.create_timer(timer_period_temp_data, self.get_temp_data_callback)

		
		timer_period_temp_command = float(500)/1000  # seconds
		self.get_temp_command_timer = self.create_timer(timer_period_temp_command, self.send_temp_command_callback)

	################################### GET SENSORS DATA ################################################
	def get_temp_data_callback(self):
		msg = String()
		self.temp_data = seradapt.writeReadSerial(self.ser, 'g\r')

		data_array = self.temp_data.split(" ")

		if(len(data_array) >= 6):
			self.temp_tempBattery = data_array[0]
			self.temp_tempDCDC = data_array[1]
			self.temp_tempOutside = data_array[4]
			self.temp_humidityOutside = data_array[5]

			msg.data = self.temp_tempBattery
			self.temp_tempBattery_publisher.publish(msg)

			msg.data = self.temp_tempDCDC
			self.temp_tempDCDC_publisher.publish(msg)

			msg.data = self.temp_tempOutside
			self.temp_tempOutside_publisher.publish(msg)

			msg.data = self.temp_humidityOutside
			self.temp_humidityOutside_publisher.publish(msg)

	################################### SEND MOTOR SPEEED COMMANDS ################################################

	def send_temp_command_callback(self):
		if not self.currentCommand == "":
			seradapt.writeReadSerial(self.ser, self.currentCommand)
			#self.get_logger().info('<- "%s"' % self.currentCommand.encode('utf-8'))
			self.currentCommand = ""
		

	################################### MOTOR COMMAND LISTENER CALLBACK ################################################

	def temp_command_listener(self, msg):
		#self.get_logger().info('-> "%s"' % msg.data.encode('utf-8'))

		if not msg.data == self.currentCommand:
			self.lastCommand = self.currentCommand
			self.currentCommand = msg.data
		else:
			self.currentCommand == ""


def main(args=None):
	rclpy.init(args=args)

	port = seradapt.loadUSB("TEM")

	tempPrintBridge = TempPrintBridge(port)
	rclpy.spin(tempPrintBridge)
	tempPrintBridge.destroy_node()
	rclpy.shutdown()
