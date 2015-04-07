#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  read_odom_node.py
#  
#  Copyright 2014 Shibata-Lab <shibata-lab@shibatalab-X500H>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def odometryCb(msg):
	#print msg.pose.pose
	quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
	euler = euler_from_quaternion(quaternion)
	#print euler
	robot_pose = [msg.pose.pose.position.x,msg.pose.pose.position.y,euler[2]]
	print robot_pose[0],robot_pose[1],robot_pose[2] 

if __name__ == '__main__':
	rospy.init_node('odometry',anonymous = True)
	rospy.Subscriber('/pose',Odometry,odometryCb)
	rospy.spin()
