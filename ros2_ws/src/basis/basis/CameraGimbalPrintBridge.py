# CameraGimbalPrintBridge
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


class CameraGimbalPrintBridge(Node):

	def __init__(self, port):
		super().__init__('CameraGimbalPrintBridge')

		self.currentCommand = ""
		self.lastCommand = ""

		self.ser = seradapt.openSerial(port)
   
		################################### GIMBAL COMMAND LISTENER ################################################
		self.camera_gimbal_command_subscription = self.create_subscription(
			String,
			'camera_gimbal',
			self.camera_gimbal_command_listener,
			10)
		self.camera_gimbal_command_subscription  # prevent unused variable warning
   
    
    ################################### GIMBAL SEND COMMAND TIMER ################################################

		timer_period_gimbal_command = float(100)/1000  # seconds
		self.gimbal_command_timer = self.create_timer(timer_period_gimbal_command, self.send_gimbal_command_callback)


	def send_gimbal_command_callback(self):
		if self.lastCommand != "":
			#print("test ser: " + self.lastCommand)
			seradapt.writeReadSerial(self.ser, self.lastCommand + '\r')
			self.lastCommand = ""

	################################### GIMBAL ORIENTATION LISTENER ################################################


	def camera_gimbal_command_listener(self, msg):
		#print("ser2: " + msg.data + '\r')
		self.lastCommand = msg.data




def main(args=None):
	rclpy.init(args=args)

	port = seradapt.loadUSB("SER")

	cameraGimbalPrintBridge = CameraGimbalPrintBridge(port)

	rclpy.spin(cameraGimbalPrintBridge)

	cameraGimbalPrintBridge.destroy_node()

	rclpy.shutdown()

