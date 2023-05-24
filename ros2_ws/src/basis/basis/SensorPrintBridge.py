# SensorPrintBridge
#	Send Controll commands to Sensor Print
# and get periodicly data from Sensor Print
# 
# V1.1
# 
# 20.4.2023
# Benjamin Zbinden

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

#Custom imports
import basis.SerialAdapter as seradapt


class SensorPrintBridge(Node):

	def __init__(self, port):
		super().__init__('SensorPrintBridge')

		self.currentCommand = ""
		self.lastCommand = ""

		self.ser = seradapt.openSerial(port)

		self.sensor_gyro_publisher 				  =   self.create_publisher(String, 'sensor_gyro', 10)
		self.sensor_acceleration_publisher  =   self.create_publisher(String, 'sensor_acceleration', 10)
		self.sensor_co2_publisher 				  =   self.create_publisher(String, 'sensor_co2', 10)
		self.sensor_inputs_publisher 			  =   self.create_publisher(String, 'sensor_inputs', 10)
		self.sensor_notstop_publisher			  =   self.create_publisher(String, 'robot_notstop', 10)
		self.sensor_distance_publisher 		  =   self.create_publisher(String, 'sensor_distance', 10)

		################################### SENSOR COMMAND LISTENER ################################################
		self.sensor_command_subscription = self.create_subscription(
			String,
			'sensor_command',
			self.sensor_command_listener,
			10)
		self.sensor_command_subscription  # prevent unused variable warning


		################################### TIMERS FOR CALLING SENSOR BOARD ###########################################
		timer_period_sensor_gyro = float(100)/1000  # seconds
		self.get_sensor_gyro_timer = self.create_timer(timer_period_sensor_gyro, self.get_sensor_gyro_callback)

		timer_period_sensor_co2 = float(1000)/1000  # seconds
		self.get_sensor_co2_timer = self.create_timer(timer_period_sensor_co2, self.get_sensor_co2_callback)

		timer_period_sensor_inputs = float(300)/1000  # seconds
		self.get_sensor_inputs_timer = self.create_timer(timer_period_sensor_inputs, self.get_sensor_inputs_callback)

		timer_period_sensor_distance = float(500)/1000  # seconds
		self.get_sensor_distance_timer = self.create_timer(timer_period_sensor_distance, self.get_sensor_distance_callback)


		timer_period_sensor_command = float(500)/1000  # seconds
		self.get_sensor_command_timer = self.create_timer(timer_period_sensor_command, self.send_sensor_command_callback)
	 

	################################### GET SENSORS DATA ################################################
	def get_sensor_gyro_callback(self):
		msg = String()
		self.sensor_gyro = seradapt.writeReadSerial(self.ser, 'g\r')
		self.sensor_acceleration = seradapt.writeReadSerial(self.ser, 'a\r')

		msg.data = self.sensor_gyro
		self.sensor_gyro_publisher.publish(msg)

		msg.data = self.sensor_acceleration
		self.sensor_acceleration_publisher.publish(msg)
	
	def get_sensor_co2_callback(self):
		msg = String()
		self.sensor_co2 = seradapt.writeReadSerial(self.ser, 'c\r')

		msg.data = self.sensor_co2
		self.sensor_co2_publisher.publish(msg)

	def get_sensor_inputs_callback(self):
		msg = String()
		self.sensor_inputs = seradapt.writeReadSerial(self.ser, 'x\r')

		msg.data = self.sensor_inputs
		self.sensor_inputs_publisher.publish(msg)
		
		msg_array = msg.data.split(" ")
		if(len(msg_array) >= 5):
			if(msg_array[0] == "0" and msg_array[1] == "0"):
				msg.data = "1"
			else:
				msg.data = "0"
			
			self.sensor_notstop_publisher.publish(msg)

	def get_sensor_distance_callback(self):
		msg = String()
		self.sensor_distance = seradapt.writeReadSerial(self.ser, 'd\r')

		msg.data = self.sensor_distance
		self.sensor_distance_publisher.publish(msg)

	################################### SEND MOTOR SPEEED COMMANDS ################################################

	def send_sensor_command_callback(self):
		if not self.currentCommand == "":
			seradapt.writeReadSerial(self.ser, self.currentCommand)
			#self.get_logger().info('<- "%s"' % self.currentCommand.encode('utf-8'))
			self.currentCommand = ""
		else:
			seradapt.writeReadSerial(self.ser, 'p\r')
		

	################################### MOTOR COMMAND LISTENER CALLBACK ################################################

	def sensor_command_listener(self, msg):
		#self.get_logger().info('-> "%s"' % msg.data.encode('utf-8'))

		if not msg.data == self.currentCommand:
			self.lastCommand = self.currentCommand
			self.currentCommand = msg.data
		else:
			self.currentCommand == ""


def main(args=None):
	rclpy.init(args=args)

	port = seradapt.loadUSB("SEN")

	sensorPrintBridge = SensorPrintBridge(port)
	rclpy.spin(sensorPrintBridge)
	sensorPrintBridge.destroy_node()
	rclpy.shutdown()
