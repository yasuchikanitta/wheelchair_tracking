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
		self.cmd_vel = rospy.Publisher("cmd_vel",Twist,queue_size=1)

		rospy.Subscriber("/go", Int32, self.go)
		rospy.Subscriber("/nothing", Int32, self.nothing)
		rospy.Subscriber("/person",numpy_msg(Floats), self.processingperson)
		rospy.Subscriber("/joystick",numpy_msg(Floats),self.joystick)
		rospy.Subscriber("/ar_marker_rec/object_info", String, self.callback_re)
		rospy.on_shutdown(self.shutdown)

		self.non = 0
		self.go = 2
		self.joy = np.array([0.0,0.0])
		self.pvel = Twist()
		self.kvel = Twist()
		self.Ka = 2.0
		#self.Ka = 1.0
		#self.Kx = 0.5
		self.Kx = 0.7
		self.person = np.array([])
		self.obj_info = []


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
		#for o in obj_info:
			#self.re_position = o["position"]
			#print(self.re_position)
			#print( "label:", o["label"] )
			#print( "lefttop:", o["lefttop"] )
			#print( "rightbottom:", o["rightbottom"] )
			#print( "position:", o["position"] )
			#print("距離",o["position"][2])
			#print( "position_mindepth:", o["position_mindepth"] )
			#print( "position_center:", o["position_center"] )
			#print( "position_bottom:", o["position_bottom"] )
			#print( "-----------" )

	def run(self):
		rate = rospy.Rate(10)
		vrl = Twist()
		#l = 0.0
		d = 0
		#theta = np.array([0.0,0.0])
		T = str(time.time())
		print(T)
		while not rospy.is_shutdown():
			for o in self.obj_info:
				l = o["position"][2]
				Theta = o["position"][0]
				print(Theta)
				print(l)

				#vx = -self.Kx*l*0.4
				vx = -self.Kx*l*0.4
				va = -self.Ka*Theta*0.4

				if l>0.8:
					self.kvel.linear.x = vx
						
				elif l<0.7:
					self.kvel.linear.x = -vx

				else:
					self.kvel.linear.x = 0.0

				self.kvel.angular.z = va
				print("vx:" +str(self.kvel.linear.x))
				print("va:"+str(self.kvel.angular.z))
				self.cmd_vel.publish(self.kvel)

	def shutdown(self):
		self.pvel.linear.x = 0.0
		self.pvel.angular.z = 0.0
		self.cmd_vel.publish(self.pvel)


if __name__ == '__main__':
	rospy.init_node('follow')
	rospy.loginfo("tracking start")
	tracking().run()
