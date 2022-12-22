#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import time
import rospy
import numpy as np
import pylab as pl
import cv2
#import cv as cv
import csv
from numpy.linalg import solve
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
#from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class tracking():
	def __init__(self):
		rospy.on_shutdown(self.shutdown)
		self.wtracking = rospy.Publisher("/wtracking",Twist,queue_size=1)
		self.pvel = Twist()
		rospy.Subscriber("person", numpy_msg(Floats), self.processingperson)
		self.Ka = 3.5
		self.Kx = 0.5
		self.Pvel_vx = 0.0
		self.Pvel_va = 0.0
		self.person = np.array([])

	def processingperson(self,databuf):
		person = np.array(databuf.data)
		self.person = person.reshape(3,2)


	def run(self):
		resi = 0
		rate = rospy.Rate(10)
		time.sleep(3)
		while not rospy.is_shutdown():
			person = self.person
			l=np.sqrt(person[0,0]**2+person[0,1]**2)
			theta = np.arctan2(person[0,1],person[0,0])
			
			vx=self.Kx*l
			va=-self.Ka*theta

			if l>1.0:
				self.pvel.linear.x = vx
			elif l<0.95:
				self.pvel.linear.x = 0.0
				self.pvel.angular.z = 0.0

			self.pvel.angular.z = va
			print ("vx:" +str(self.pvel.linear.x))
			print ("va:"+str(self.pvel.angular.z))
			self.wtracking.publish(self.pvel)
			rate.sleep()

	def shutdown(self):
		self.pvel.linear.x = 0.0
		self.pvel.angular.z = 0.0
		self.wtracking.publish(self.pvel)

if __name__ == '__main__':
	rospy.init_node('tracking')
	rospy.loginfo("tracking start")
	tracking().run()