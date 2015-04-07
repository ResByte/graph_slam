#!/usr/bin/env python
import roslib 
import rospy

import tf 
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Point, Quaternion


def handle_robot_pose(poseMsg,currTime):
	br = tf.TransformBroadcaster()  # create a broadcaster
	theta = tf.transformations.quaternion_from_euler(0,0,poseMsg[2])
	
	odom = Odometry()
	odom.header.stamp = currTime
	odom.header.frame_id = "odom" 
	#set the position
	odom.pose.pose.position = Point(poseMsg[0], poseMsg[1], 0)
	odom.pose.pose.orientation = theta
	#set the velocity
	odom.child_frame_id = "base_link"
	#odom.twist.twist.linear.x = 0
	#odom.twist.twist.linear.y = 0
	#odom.twist.twist.angular.z = 0
	
	odomPub = rospy.Publisher('odom',Odometry,queue_size = 1)
	odomPub.publish(odom)
	br.sendTransform((poseMsg[0],poseMsg[1],0.0),theta,currTime,"base_link","odom") # publish it on ros
	#create odometry msg
	 
	print "transform sent"

def handle_laser_scan(laserMsg,currTime):
	scan = LaserScan()
	scan.header.stamp = currTime
	scan.header.frame_id = "base_link"
	scan.angle_min = -1.57
	scan.angle_max = 1.57
	scan.angle_increment = 3.14/181
	scan.time_increment = (1/0.5)/181
	scan.range_min = 0.0
	scan.range_max = 10.0
	scan.ranges = [0.0 for i in xrange(len(laserMsg)-1)]
	scan.intensities = [0.0 for i in xrange(len(laserMsg)-1)]
	for i in xrange(len(laserMsg)-1):
		scan.ranges[i] =laserMsg[i]
		scan.intensities[i]= 100
	laserPub =  rospy.Publisher('scan',LaserScan,queue_size = 1)
	laserPub.publish(scan)
	rospy.sleep(4.0)

if __name__ == '__main__':
	rospy.init_node('orebro_tf_broadcaster') # initialize ros node 
	
	with open(sys.argv[1], 'r') as dataFile:
		count = 1
		for line in dataFile:
			a = line.strip().split()
			timeNow =rospy.Time.now()
			if count >= 5 and count< 20 :
				print count
				poseX = float(a[1])/100.0
				poseY =  float(a[2])/100.0
				poseTheta = float(a[3])
				handle_robot_pose([poseX,poseY,poseTheta],timeNow)
				laserData =[]
				for i in xrange(180):
					laserData.append(float(a[i+4])/100.0)
				handle_laser_scan(laserData,timeNow)

			count+=1


