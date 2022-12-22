#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -*- Python -*-
from __future__ import print_function, unicode_literals
import yaml

import sys
import time
import rospy
import numpy as np
import pylab as pl
from numpy.linalg import solve
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int32

class tracking():
	def __init__(self):
		self.wtracking = rospy.Publisher("/wtracking",Twist,queue_size=1)
		#self.cmd_vel = rospy.Publisher("cmd_vel",Twist,queue_size=1)

		#rospy.Subscriber("/go", Int32, self.go)
		#rospy.Subscriber("/nothing", Int32, self.nothing)
		#rospy.Subscriber("/person",numpy_msg(Floats), self.processingperson)
		#rospy.Subscriber("/joystick",numpy_msg(Floats),self.joystick)
		rospy.Subscriber("/ar_marker_rec/object_info", String, self.callback_re)
		rospy.on_shutdown(self.shutdown)

		self.non = 0
		self.go = 2
		self.joy = np.array([0.0,0.0])
		self.pvel = Twist()
		self.kvel = Twist()
		self.Ka = 5.0
		self.Kx = 0.2
		self.person = np.array([])
		self.obj_info = []
		self.obj_info_r = {}
		self.obj_info_l = {}
		self.l = 0.0
		self.Theta = 0.0
		self.threshold = 0
		self.a = 0.0
		self.b = 0.0


	def processingperson(self,databuf):
		person = np.array(databuf.data)
		self.person = person.reshape(3,2)

	def go(self,letsgo):
		self.go = letsgo.data

	def nothing(self,nothing):
		self.non = nothing.data


	def joystick(self,joystick):
		self.joy = joystick.data

	def callback_re(self,data):
		self.obj_info = yaml.full_load(data.data)
		#print(self.obj_info)
		self.obj_info_r = self.obj_info[1]
		self.l = self.obj_info_r["position"][2]
		#print(self.l)
		#print(self.obj_info_r , type(self.obj_info_r))
		self.obj_info_l = self.obj_info[0]
		self.Theta = (self.obj_info_l["position"][2] - self.obj_info_r["position"][2])
		self.threshold = self.obj_info_r["position"][0] + self.obj_info_l["position"][0]
		print(self.Theta)
		self.a = self.obj_info_r["position"][2]
		self.b = self.obj_info_l["position"][2]

	def run(self):
		rate = rospy.Rate(10)
		vrl = Twist()
		d = 0
		#l = 0.0
		#theta = np.array([0.0,0.0])
		T = str(time.time())
		print(T)
		while not rospy.is_shutdown():
			l = self.l
			Theta = self.Theta
			threshold = self.threshold
			#print("l=",l)
			#print("Theta=",Theta)
			#print("threshold=",threshold)

			#vx = -self.Kx*l*0.4
			vx = self.Kx*l
			va = -self.Ka*Theta

			if l>1.25:
				self.kvel.linear.x = vx
						
			elif l<1.2:
				self.kvel.linear.x = 0.0

			else:
				self.kvel.linear.x = 0.0

			self.kvel.angular.z = va
			print("vx:" +str(self.kvel.linear.x))
			print("va:"+str(self.kvel.angular.z))
			print(self.Theta , self.a , self.b)
			self.wtracking.publish(self.kvel)

			#self.cmd_vel.publish(self.kvel)

	def shutdown(self):
		self.pvel.linear.x = 0.0
		self.pvel.angular.z = 0.0
		self.wtracking.publish(self.pvel)


if __name__ == '__main__':
	rospy.init_node('tracking_realsense')
	rospy.loginfo("tracking start")
	tracking().run()
