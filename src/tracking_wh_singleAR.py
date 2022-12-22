#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -*- Python -*-
from __future__ import print_function, unicode_literals
import yaml

import sys
import time
import rospy
import numpy as np
import math
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
		rospy.Subscriber("/ar_marker_rec/object_info", String, self.callback_re)
		rospy.on_shutdown(self.shutdown)
		self.kvel = Twist()
		self.Ka = 0.5
		self.Kx = 0.3
		self.obj_info = []

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
		M_l = 0.0
		M_th = 0.0
		M1_l = 0.0
		M1_th = 0.0

		g_l = 1.0
		g_th = 0.0
		e_l = 0.0
		e1_l = 0.0
		e2_l = 0.0
		e_th = 0.0
		e1_th = 0.0
		e2_th = 0.0

		kp = 0.1
		ki = 0.1
		kd = 0.1

		M_l_list = []
		M_th_list = []
		#while not rospy.is_shutdown():
		for i in range(1,100):
			for o in self.obj_info:
				l = o["position"][2]
				Theta = math.atan2(o["position"][0],o["position"][2])

				M1_l = M_l
				M1_th = M_th
				e2_l = e1_l
				e1_l = e_l
				e2_th = e1_th
				e1_th = e_th

				e_l = g_l - l
				e_th = g_th - Theta

				M_l = M1_l + kp * (e_l - e1_l) + ki * e_l + kd * ((e_l - e1_l) - (e1_l - e2_l))
				M_th = M1_th + kp * (e_th - e1_th) + ki * e_th + kd * ((e_th - e1_th) - (e1_th - e2_th))
				M_l_list.append(M_l)
				M_th_list.append(M_th)

				rospy.loginfo(len(M_l_list))
				rospy.loginfo(len(M_th_list))
				vx = M_l
				va = M_th

				self.kvel.linear.x = vx
				self.kvel.angular.z = va

				#rospy.loginfo("vx:" +str(self.kvel.linear.x))
				#rospy.loginfo("va:"+str(self.kvel.angular.z))
				self.wtracking.publish(self.kvel)

	def shutdown(self):
		self.kvel.linear.x = 0.0
		self.kvel.angular.z = 0.0
		self.wtracking.publish(self.kvel)


if __name__ == '__main__':
	rospy.init_node('tracking_wh')
	rospy.loginfo("tracking start")
	tracking().run()
