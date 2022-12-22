#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -*- Python -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class contrall():
	def __init__(self):
		self.cmd_vel = rospy.Publisher("cmd_vel",Twist,queue_size=1)
		rospy.Subscriber("joy", Joy , self.joy_callback)

		self.linear_x = 0.0
		self.angular_zr = 0.0
		self.angular_zl = 0.0
		self.change_speed = 0.0
	
	def joy_callback(self, joy_msg) :
		self.linear_x = joy_msg.axes[1]
		#self.angular_zr = joy_msg.buttons[7] #buffalo
		#self.angular_zl = joy_msg.buttons[6] #buffalo
		self.angular_zr = joy_msg.buttons[5] #logicool
		self.angular_zl = joy_msg.buttons[4] #logicool
		self.change_speed = joy_msg.buttons[1]

	def main(self):
		rate = rospy.Rate(10)
		self.twist = Twist()
		while not rospy.is_shutdown():
			if self.change_speed == 1:
				self.twist.linear.x = int(self.linear_x) * 0.4
				self.twist.angular.z = int(self.angular_zl - self.angular_zr) * 0.5
			
			else:
				self.twist.linear.x = int(self.linear_x) * 0.2
				self.twist.angular.z = int(self.angular_zl - self.angular_zr) * 0.3



			self.cmd_vel.publish(self.twist) 
			rate.sleep()

def shutdown(self):
		self.twist.linear.x = 0.0
		self.twist.angular.z = 0.0
		self.cmd_vel.publish(self.twist)

if __name__ == '__main__':
	rospy.init_node('joy_sub')
	rospy.loginfo("joy_sub")
	contrall().main()
