#!/usr/bin/env python
import rclpy
import time
from rclpy.qos import (
	QoSDurabilityPolicy,
	QoSHistoryPolicy,
	QoSProfile,
	QoSReliabilityPolicy,
)
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

from std_srvs.srv import Trigger

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class ServoClientNode(Node):

	def __init__(self):
		super().__init__('teleop_twist_keyboard')
		self.declare_parameter("servo_control", rclpy.Parameter.Type.STRING)
		param_servo_control = self.get_parameter("servo_control").value
		if param_servo_control in ["sync","async", "none"]:
			self.servo_control_mode = param_servo_control
		else:
			print("Invalid parameter value (servo_control): must be one of: 'sync', 'async', 'none'")
			exit()
		print("parameter used:",param_servo_control)

		self.__is_enabled = False

		if self.servo_control_mode == "none":
			self.__start_service = None
			self.__stop_service = None
		elif self.servo_control_mode == "sync":
			pass
		else:
			self.__start_service = self.create_client(Trigger, '/servo_node/start_servo')
			self.__stop_service = self.create_client(Trigger, '/servo_node/stop_servo')
			while not self.__start_service.wait_for_service(timeout_sec=1.0):
				self.get_logger().info('start_service not available, waiting again...')
			while not self.__stop_service.wait_for_service(timeout_sec=1.0):
				self.get_logger().info('stop_service not available, waiting again...')

		self.req = Trigger.Request()
		

	def request_servo_start(self):
		result = self.__start_service.call(self.req)
		if not result.success:
			self._node.get_logger().error(
				f"MoveIt Servo could not be enabled. ({result.message})"
			)
		self.__is_enabled = result.success

	def request_servo_stop(self):
		result = self.__stop_service.call(self.req)
		if not result.success:
			self._node.get_logger().error(
				f"MoveIt Servo could not be disabled. ({result.message})"
			)
		self.__is_enabled = False
	
	def request_servo_start_async(self):
		self.future = self.__start_service.call_async(self.req)
		

	def request_servo_stop_async(self):
		self.future = self.__stop_service.call_async(self.req)
	
	def teleop(self):
		qos_profile = QoSProfile(depth=20)
		pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', qos_profile)

		speed = 0.5
		turn = 1.0
		x = 0
		y = 0
		z = 0
		th = 0
		status = 0

		try:
			print(msg)
			print(vels(speed,turn))
			while(1):
				key = getKey()
				if key in moveBindings.keys():
					x = moveBindings[key][0]
					y = moveBindings[key][1]
					z = moveBindings[key][2]
					th = moveBindings[key][3]
				elif key in speedBindings.keys():
					speed = speed * speedBindings[key][0]
					turn = turn * speedBindings[key][1]

					print(vels(speed,turn))
					if (status == 14):
						print(msg)
					status = (status + 1) % 15
				else:
					x = 0
					y = 0
					z = 0
					th = 0
					if (key == '\x03'):
						break

				twist_msg = TwistStamped()
				twist = twist_msg.twist
				twist_msg.header.stamp = self.get_clock().now().to_msg()
				twist_msg.header.frame_id = "panda_link0"
				twist.linear.x = x*speed
				twist.linear.y = y*speed
				twist.linear.z = z*speed
				twist.angular.x = 0.0
				twist.angular.y = 0.0
				twist.angular.z = th*turn
				pub.publish(twist_msg)

		except Exception as e:
			print(e)

		finally:
			twist_msg = TwistStamped()
			twist = twist_msg.twist
			twist_msg.header.stamp = self.get_clock().now().to_msg()
			twist_msg.header.frame_id = "panda_link0"
			twist.linear.x = 0.0
			twist.linear.y = 0.0
			twist.linear.z = 0.0
			twist.angular.x = 0.0
			twist.angular.y = 0.0
			twist.angular.z = 0.0
			pub.publish(twist_msg)

			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
		   }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
		  }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)


def main():	
	rclpy.init()
	client = ServoClientNode()

	
	if client.servo_control_mode == "async":
		print("enabling Servo")
		client.request_servo_start_async()
		while rclpy.ok():
			time.sleep(0.1)
			if client.future.done():
				try:
					response = client.future.result()
					print("Servo started succesfully", response)
				except Exception as e:
					print("Service call failed", e)
	elif client.servo_control_mode == "sync":
		raise NotImplementedError()	
	print("starting teleop")
	client.teleop()
	

	if client.servo_control_mode == "async":
		print("disabling Servo")
		client.request_servo_stop_async()
		while rclpy.ok():
			time.sleep(0.1)
			if client.future.done():
				try:
					response = client.future.result()
					print("Servo stopped succesfully", response)
				except Exception as e:
					print(f"Service call failed", e)
	elif client.servo_control_mode == "sync":
		raise NotImplementedError()	
	
	client.destroy_node()
	rclpy.shutdown()


	