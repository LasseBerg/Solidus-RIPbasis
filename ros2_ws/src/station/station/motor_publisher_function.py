# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

## custom imports
import math
import time
from scipy.interpolate import Akima1DInterpolator
from evdev import InputDevice

last_LS_VERTICAL = 0;
last_RS_VERTICAL = 0;
last_LS_HORIZONTAL = 0;
last_RS_HORIZONTAL = 0;

MAX_MOTOR_SPEED = 18000
MAX_VECTOR_AMOUNT = 32750
CONTROLLER_EVENTS = '/dev/input/event19'

LIMIT_STICK_INPUTS = 500 # in ms
LIMIT_STICK_VALUE_DIFF = 1000
last_stick_handle = time.monotonic(); # TODO make sure the value doesnt overflow after a long time

STICK_TOLERANCE = 1000

# controller codes for key mapping
LS_HORIZONTAL = 1
LS_VERTICAL = 0
RS_HORIZONTAL = 3
RS_VERTICAL = 4
A = 304
B = 305


def readController(publisher):
	gamepad = InputDevice(CONTROLLER_EVENTS)
	for event in gamepad.read_loop():
		if(event.type != 00):
			if event.code in switch:
				msg = String()
				command = switch[event.code](event.value) # Call the function mapped to the key
				if(command is not None):
					msg.data = command + "\r\n"
					print("Sending: " + msg.data)
					publisher.publish(msg)
			else:
				print("Key not found in switch")


### CONTROLLER HANDLERS ###
def handle_LS_HORIZONTAL(val):
	if check_last_stick_handle(val, "HORIZONTAL"):
		global last_LS_HORIZONTAL, last_LS_VERTICAL
		last_LS_HORIZONTAL = val;
		return calculate_motors_speed(last_LS_HORIZONTAL, last_LS_VERTICAL)


def handle_LS_VERTICAL(val):
	if check_last_stick_handle(val, "VERTICAL"):
		global last_LS_HORIZONTAL, last_LS_VERTICAL
		last_LS_VERTICAL = val;
		return calculate_motors_speed(last_LS_HORIZONTAL, last_LS_VERTICAL)


def handle_A(val):
	#if val:
		#print("A pressed")
	#else:
		#print("A released")

	return "p"


def handle_B(val):
	if val:
		return "s 1 1 1 1"
	else:
		return "s 0 0 0 0"

	


switch = {
	LS_HORIZONTAL: handle_LS_HORIZONTAL,
	LS_VERTICAL: handle_LS_VERTICAL,
	A: handle_A,
	B: handle_B,
}
### END CONTROLLER HANDLERS ###

##### HELPER FUNCTIONS #####

# AKIMA INTERPOLATION
def estimate_motor_factors(x_value, side):
	# Given data (see documentation https://hackmd.io/-sj5lnRURdqnvW6N3OMhYg?both#Defining-the-motor-behaviour)
	x_data = [-3.1, -2.3, -1.5, -0.7, 0, 0.7, 1.5, 2.3, 3.1]
	y_data = [1, 1, 1, -1, -1, 0, -1, 0, 1]

	# reverse the array for the left side
	if(side == 'left'):
		y_data.reverse()

	# Create Akima cubic spline interpolator
	interpolator = Akima1DInterpolator(x_data, y_data)

	# Estimate y value at x_value
	y_value = interpolator(x_value)

	return y_value


# CONVERT THE ONE RANGE TO ANOTHER
def convert_range(value, old_min, old_max, new_min, new_max):
	# Calculate the old and new ranges
	old_range = old_max - old_min
	new_range = new_max - new_min

	# Convert the value to the new range
	new_value = (((value - old_min) * new_range) / old_range) + new_min

	# Return the new value
	return int(new_value)


# LIMITS THE MOTOR FACTORS FROM -1 TO 1
def limit_motor_factor(factor_value):
	# limit motor factors from -1 to 1
	if factor_value < -1:
		factor_value = -1
	elif factor_value > 1:
		factor_value = 1

	return factor_value


# CALCULATE THE SPEED AND DIRECTION OF THE MOTORS
def calculate_motors_speed(last_LS_HORIZONTAL, last_LS_VERTICAL):
	speed = round(math.sqrt(last_LS_HORIZONTAL**2 + last_LS_VERTICAL**2), 1)
	direction = round(math.atan2(last_LS_HORIZONTAL, last_LS_VERTICAL), 1)

	ResultLeftMotors = estimate_motor_factors(direction, 'left')
	ResultRightMotors = estimate_motor_factors(direction, 'right')

	ResultLeftMotors = limit_motor_factor(ResultLeftMotors);
	ResultRightMotors = limit_motor_factor(ResultRightMotors);

	ResultLeftMotors = ResultLeftMotors * speed
	ResultRightMotors = ResultRightMotors * speed

	ResultLeftMotors = convert_range(ResultLeftMotors, -MAX_VECTOR_AMOUNT, MAX_VECTOR_AMOUNT, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED)
	ResultRightMotors = convert_range(ResultRightMotors, -MAX_VECTOR_AMOUNT, MAX_VECTOR_AMOUNT, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED)

	# invert left wheels, since they are physically rotated 180deg compared to the right motors
	#ResultLeftMotors = ResultLeftMotors * -1
	if (ResultLeftMotors > -STICK_TOLERANCE and ResultLeftMotors < STICK_TOLERANCE):
		ResultLeftMotors = 1
		return "s 1 1 1 1"

	if (ResultRightMotors > -STICK_TOLERANCE and ResultRightMotors < STICK_TOLERANCE):
		ResultRightMotors = 1
		return "s 1 1 1 1"

	cmd = "m {} {} {} {}".format(ResultLeftMotors, ResultRightMotors, ResultLeftMotors, ResultRightMotors)
	return cmd


# LIMITS THE AMOUNT OF STICK INPUTS
def check_last_stick_handle(current_val, direction):
	if(direction == "VERTICAL") and (abs(current_val - last_LS_VERTICAL) > LIMIT_STICK_VALUE_DIFF):
		return True		
	
	if(direction == "HORIZONTAL") and (abs(current_val - last_LS_HORIZONTAL) > LIMIT_STICK_VALUE_DIFF):
		return True		
	
	global last_stick_handle

	current_time = time.monotonic()
	if(current_time - last_stick_handle > (LIMIT_STICK_INPUTS / 1000)):
		last_stick_handle = current_time
		return True
	else:
		return False

##### END HELPER FUNCTIONS #####


class MotorPublisher(Node):

	def __init__(self):
		super().__init__('MotorPublisher')
		self.publisher_ = self.create_publisher(String, "motor_command", 10)

		readController(self.publisher_)

		#timer_period = 0.1  # seconds
		#self.timer = self.create_timer(timer_period, self.timer_callback)
		#self.i = 0


def main(args=None):
	rclpy.init(args=args)

	motorPublisher = MotorPublisher()

	rclpy.spin(motorPublisher)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	motorPublisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
