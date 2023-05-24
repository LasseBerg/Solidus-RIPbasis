# MotorPrintBridge
#	Send Controll commands to motor controller
# and get periodicly data from motors
# 
# V1.1
# 
# 20.4.2023
# Lars Berg / Benjamin Zbinden

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

#Custom imports
import basis.SerialAdapter as seradapt


class MotorPrintBridge(Node):

	def __init__(self, port):
		super().__init__('MotorPrintBridge')

		self.currentCommand = ""
		self.lastCommand = ""
		self.timeLastCommand = 0
		
		self.robot_notstop = False
		self.gui_notstop = False

		self.ser = seradapt.openSerial(port)
		self.motor_volt_publisher = self.create_publisher(String, 'motor_volt', 10)
		self.motor_temp_publisher = self.create_publisher(String, 'motor_temp', 10)
		self.motor_error_publisher = self.create_publisher(String, 'motor_error', 10)

		################################### MOTOR COMMAND LISTENER ################################################
		self.motor_command_subscription = self.create_subscription(
			String,
			'motor_command',
			self.motor_command_listener,
			10)
		self.motor_command_subscription  # prevent unused variable warning
		
		################################### ROBOT NOTSTOPP COMMAND LISTENER ##########################
		self.robot_notstop_command_subscription = self.create_subscription(
			String,
			'robot_notstop',
			self.robot_notstop_command_listener,
			10)
		self.robot_notstop_command_subscription  # prevent unused variable warning
	 
		################################### GUI NOTSTOPP COMMAND LISTENER ##########################
		self.gui_notstop_command_subscription = self.create_subscription(
			String,
			'gui_notstop',
			self.gui_notstop_command_listener,
			10)
		self.gui_notstop_command_subscription  # prevent unused variable warning

		################################### TIMERS FOR CALLING CAN BOARD ###########################################
		timer_period_motor_error = float(1500)/1000  # seconds
		self.motor_get_error_timer = self.create_timer(timer_period_motor_error, self.get_motor_error_callback)

		timer_period_motor_temp = float(1000)/1000  # seconds
		self.motor_get_temp_timer = self.create_timer(timer_period_motor_temp, self.get_motor_temp_callback)

		timer_period_motor_volt = float(1000)/1000  # seconds
		self.motor_get_volt_timer = self.create_timer(timer_period_motor_volt, self.get_motor_volt_callback)


		self.timer_period_motor_command = float(10)/1000  # seconds
		self.motor_command_timer = self.create_timer(self.timer_period_motor_command, self.send_motor_command_callback)


	################################### GET MOTOR DATA ################################################
	def get_motor_error_callback(self):
		self.motor_error = seradapt.writeReadSerial(self.ser, 'e\r')
		#print(f"Current Motor Errors: {self.motor_error}")

		msg = String()
		msg.data = self.motor_temp
		self.motor_temp_publisher.publish(msg)
	
	def get_motor_temp_callback(self):
		self.motor_temp = seradapt.writeReadSerial(self.ser, 't\r')
		#print(f"Current Motor Temperatur: {self.motor_temp}")

		msg = String()
		msg.data = self.motor_temp
		self.motor_temp_publisher.publish(msg)

	def get_motor_volt_callback(self):
		self.motor_volt = seradapt.writeReadSerial(self.ser, 'v\r')
		#print(f"Current Motor Voltage: {self.motor_volt}")

		msg = String()
		if(len(self.motor_volt) >= 3):
			msg.data = self.motor_volt
			self.motor_volt_publisher.publish(msg)

	################################### SEND MOTOR SPEED COMMANDS (CYCLIC) ################################################

	def send_motor_command_callback(self):
		if(self.robot_notstop or self.gui_notstop):
			seradapt.writeReadSerial(self.ser, 's 1 1 1 1' + '\r')
		else:
			if not self.currentCommand == "":
				seradapt.writeReadSerial(self.ser, self.currentCommand + '\r')
				self.lastCommand = self.currentCommand
				self.currentCommand = ""
				self.timeLastCommand = 0
			else:
				self.timeLastCommand = self.timeLastCommand + self.timer_period_motor_command
				if self.timeLastCommand >= 0.5:
					seradapt.writeReadSerial(self.ser, self.lastCommand + '\r')
					self.timeLastCommand = 0

	###################################  COMMAND LISTENER CALLBACK ################################################

	def motor_command_listener(self, msg):
		if not msg.data == self.currentCommand:
			self.lastCommand = self.currentCommand
			self.currentCommand = msg.data
		else:
			self.currentCommand == ""
	
	################################### ROBOT NOTSTOPP COMMAND LISTENER CALLBACK #########################>

	def robot_notstop_command_listener(self, msg):
	
		if(msg.data == "1"):
			self.robot_notstop = True
		else:
			self.robot_notstop = False
			
	
	################################### GUI NOTSTOPP COMMAND LISTENER CALLBACK #########################>

	def gui_notstop_command_listener(self, msg):
	
		if(msg.data == "1"):
			self.gui_notstop = True
		else:
			self.gui_notstop = False



def main(args=None):
	rclpy.init(args=args)

	port = seradapt.loadUSB("CAN")

	motorPrintBridge = MotorPrintBridge(port)

	rclpy.spin(motorPrintBridge)

	motorPrintBridge.destroy_node()

	rclpy.shutdown()
