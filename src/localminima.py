#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import time
import rospy
import numpy as np
import cv2
#import cv as cv
import csv
import math
from scipy.stats import norm
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
	#init
	def __init__(self):
		self.person = rospy.Publisher("/person",numpy_msg(Floats),queue_size=1)
		self.odom = Odometry()
		rospy.Subscriber("joy", Joy, self.processing_joy)
		#rospy.Subscriber("mobile_base/events/bumper", BumperEvent,self.processing_bumper)
		rospy.Subscriber("scan1", LaserScan, self.processing_laser)#scan2からscanに変更20220818
		rospy.Subscriber("odom", Odometry, self.processing_odom)
		self.laser_xy = np.array([[]])
		self.record = np.array([[0.0,0.0],[0.0,0.0]])
		self.joyflag = 0
		self.bumperflag = 0

	#wiimote
	def processing_joy(self,datajoy):
		if datajoy.buttons[3] == 1:
			print (joyflag=1)
			self.joyflag = 1

	#kobukibumper
	def processing_bumper(self,databumper):
		print (processingbumper)
		if databumper.bumper == 0 or databumper.bumper == 2 or databumper.bumper == 1:
			print (bumperflag=1)
			self.bumperflag = 1

	#laser
	def processing_laser(self,data):
		laser_range = np.array(data.ranges)
		#−120°から-120°まで
		laser_range = laser_range[43:]

		#2cm以下の要素の位置を抽出
		self.R_min = np.argwhere(laser_range < 0.02)

		rad = np.arange(data.angle_min,data.angle_max+data.angle_increment,data.angle_increment)
		#−120°から-120°まで
		rad = rad[43:]

		#convert laserdata to xy plane
		self.laser_xy = np.array([laser_range*np.cos(rad),laser_range*np.sin(rad)]).T


	#odometry
	def processing_odom(self,data):
		self.odomdata = data



	#mainloop
	def recognition(self):
		person_center = np.array([[0.0,0.0],[0.0,0.0],[0.0,0.0]])
		#一度のみトピックodomを読み込みodom_initに入れる
		odom_init = rospy.wait_for_message("odom", Odometry)
		#記録をodomにより初期化
		self.record = np.array([[odom_init.pose.pose.position.x,odom_init.pose.pose.position.y],[odom_init.pose.pose.position.x,odom_init.pose.pose.position.y]])
		while not rospy.is_shutdown():
			if self.laser_xy.size > 0:
				laser_xy = self.laser_xy
				odomdata = self.odomdata

				#足候補のindexをリスト化
				start_index,end_index = self.listing_candidate(laser_xy.tolist())

				if len(start_index) == 0 or len(end_index) == 0:
					continue

				#確率密度関数で足の幅による重みを表現
				width_pdf = self.width_pdf(laser_xy,start_index,end_index)


				#odometry基準に変換
				laser_xy_world = self.world_transer(laser_xy,odomdata)


				#確率密度関数で足の位置による重みを表現
				position_pdf = self.position_pdf(laser_xy_world,start_index,end_index,self.record)


				#総合的な重み
				legs_pdf = width_pdf*position_pdf


				#重さが1番のインデックス
				index_max = self.choose_index(legs_pdf,1)
				if legs_pdf.size > 1:
					#重さが2番のインデックス
					index_sec = self.choose_index(legs_pdf,2)
				else:
					index_sec = index_max

				if index_max.size > 1:
					index_max = index_max[1]

				if index_sec.size > 1:
					index_sec = index_sec[1]

				start_index_max = start_index[int(index_max)]
				end_index_max = end_index[int(index_max)]
				start_index_sec = start_index[int(index_sec)]
				end_index_sec = end_index[int(index_sec)]

				
				center_max,leg_width_max = self.middle_and_distance(laser_xy[start_index_max],laser_xy[end_index_max])
				center_sec,leg_width_sec = self.middle_and_distance(laser_xy[start_index_sec],laser_xy[end_index_sec])
				center,legs_width = self.middle_and_distance(center_max,center_sec)


				if legs_pdf[index_max]>0.5:
					person_center = np.array([center_max,center_max,center_max])
					if leg_width_max < 0.18 and legs_pdf[index_sec]>0.1:
						#[人の重心,片足(重さ１番目),片足(重さ２番目)]
						person_center = np.array([center,center_max,center_sec])


				if self.bumperflag==1 or self.joyflag==1:
					self.record = np.array([[odom_init.pose.pose.position.x,odom_init.pose.pose.position.y],[odom_init.pose.pose.position.x,odom_init.pose.pose.position.y]])
					person_center = np.array([[0.0,0.0],[0.0,0.0],[0.0,0.0]])
					self.bumperflag=0
					self.joyflag=0


				#odometry基準に変換
				person_center_world = self.world_transer(person_center[1],odomdata)

				self.record = np.array([person_center_world,self.record[0]])

				print (person_center[0])
				person_center_f = person_center.flatten()
				person_center_f = np.float32(person_center_f)
				self.person.publish(person_center_f)
				


	#足候補のインデックスをリスト化
	def listing_candidate(self,laser_xy):
		R_min = self.R_min.tolist()
		end_index = []
		start_index = [0]		
		coef_r = 1
		k=0
		for i in range(len(laser_xy)-1):
			if len(R_min) > 0:
				if R_min[k] == i:
					coef_r += 1
					k += 1
					if k == len(R_min):
						k = 0
			if 0.018849*coef_r <math.sqrt((laser_xy[i][0]-laser_xy[i+1][0])**2+(laser_xy[i][1]-laser_xy[i+1][1])**2):
				start_index.append(i+1)
				end_index.append(i)	

				coef_r = 1
		start_index.pop()
		return start_index,end_index


	#確率密度関数で足の幅による重みを表現
	def width_pdf(self,laser_xy,start_index,end_index):
		l=np.sqrt((laser_xy[start_index,0]-laser_xy[end_index,0])**2+(laser_xy[start_index,1]-laser_xy[end_index,1])**2).T
		width_pdf=1.2*norm.pdf(l,0.1329,0.05)+0.84*norm.pdf(l,0.242,0.05)
		return width_pdf


	#確率密度関数で足の位置による重みを表現
	def position_pdf(self,laser_xy_world,start_index,end_index,record):
		estimation = record[0]*2-record[1]
		#足の候補の中点の座標
		middle = (laser_xy_world[start_index]+laser_xy_world[end_index])/2
		l=np.sqrt((middle[:,0]-estimation[0])**2+(middle[:,1]-estimation[1])**2).T
		position_pdf=norm.pdf(l,0.02,0.1)
		return position_pdf


	#valueの大きい方からn番のインデックスを返す。
	def choose_index(self,value,n):
		value_index_n = np.where(value==np.sort(value)[-n])[0]
		return value_index_n


	#2点(start_point,end_point)間の中間点（middle）と距離（distance）を返す
	def middle_and_distance(self,start_point,end_point):
		middle = (start_point+end_point)/2
		distance = np.sqrt((start_point[0]-end_point[0])**2+(start_point[1]-end_point[1])**2)
		return middle,distance


	#相対座標から世界座標（odometry基準）に変換
	def world_transer(self,relative_data,odom):
		odom_xy = [odom.pose.pose.position.x,odom.pose.pose.position.y]
		#回転行列の作成（kobukiのodomを使う場合は回転行列を２回かける,w=cosθ,z=sinθ）
		rotation = np.array([[odom.pose.pose.orientation.w,odom.pose.pose.orientation.z],[-odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]])
		rotation_matrix = np.dot(rotation,rotation)
		#laser_xyをodometry基準の座標に変換
		world_data = np.dot(relative_data,rotation_matrix)+odom_xy

		return world_data


if __name__ == "__main__":
	rospy.init_node('recognition')
	rospy.loginfo("node start")
	tracking().recognition()
