# FusePrintBridge
# Get periodicly data from Fuse Print
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


class FusePrintBridge(Node):

	def __init__(self, port):
		super().__init__('FusePrintBridge')

		self.ser = seradapt.openSerial(port)

		self.fuse_fusestate_publisher = self.create_publisher(String, 'fuse_fusestate', 10)


	################################### TIMERS FOR CALLING SENSOR BOARD ###########################################
		timer_period_fuse_fusestate = float(1000)/1000  # seconds
		self.get_fuse_fusestate_timer = self.create_timer(timer_period_fuse_fusestate, self.get_fuse_fusestate_callback)

	################################### GET SENSORS DATA ################################################
	
	def get_fuse_fusestate_callback(self):
		msg = String()
		self.fuse_fusestate = seradapt.writeReadSerial(self.ser, 'g\r')

		msg.data = self.fuse_fusestate
		self.fuse_fusestate_publisher.publish(msg)


def main(args=None):
	rclpy.init(args=args)

	port = seradapt.loadUSB("SIC")

	fusePrintBridge = FusePrintBridge(port)
	rclpy.spin(fusePrintBridge)
	fusePrintBridge.destroy_node()
	rclpy.shutdown()