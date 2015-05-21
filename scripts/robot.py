#!/usr/bin/env python

import roslib
import rospy
import sys 
from geometry_msgs.msg import Twist
import numpy as np 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt 
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import itertools
class Robot():
	"""This is a generic robot class to implement various machine learning algorithms and """
	def __init__(self):
		self.pose = [] #check if this needs to be initialized
		rospy.init_node('robot',anonymous = False)
		
	def odomCb(self,msg):
		#print msg.pose.pose
		quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
		euler = euler_from_quaternion(quaternion)
		#print euler
		self.pose = [msg.pose.pose.position.x,msg.pose.pose.position.y,euler[2]]
		print self.pose[0],self.pose[1],self.pose[2] 
	def odomSub(self):
		rospy.Subscriber('/odom',Odometry,self.odomCb)
	def cloudCb(self,data):
		#data_out = pc2.read_points(data, field_names=None, skip_nans=False, uvs=[[data.width, data.height]])
		#cloud = list(itertools.islice(data_out,0,100))
		cloud = np.asarray(data)
		print 
	def cloudSub(self):
		rospy.Subscriber('/camera/depth/points',PointCloud2,self.cloudCb)



if __name__ == '__main__':
	print "init_node"
	try:
		robot = Robot()
		while not rospy.is_shutdown():
			#robot.odomSub()
			robot.cloudSub()		
	except:
		rospy.loginfo("node terminated.")