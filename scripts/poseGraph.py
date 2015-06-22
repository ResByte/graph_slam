#!/usr/bin/env python
import numpy as np 
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan,PointField

        
class Robot:
    def __init__(self):
        rospy.init_node('robot',anonymous = True)
        self.pose = []
        self.laser = None
    def subs(self):
        rospy.Subscriber('/pose',Odometry,self.odometryCb)
        rospy.Subscriber('/scan',LaserScan,self.laserCb)
        rospy.spin()
    def odometryCb(self,msg):
        #print msg.pose.pose
        quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        #print euler
        self.pose = [msg.pose.pose.position.x,msg.pose.pose.position.y,euler[2]]
        print self.pose[0],self.pose[1],self.pose[2]
    def laserCb(self,msg):
        angle_min = np.rad2deg(float(msg.angle_min))
        angle_max = np.rad2deg(float(msg.angle_max))
        range_min = msg.range_min
        range_max = msg.range_max
        self.laser = []
        for i in xrange(len(msg.ranges)):
            if msg.ranges[i] != np.nan:
                if msg.ranges[i] < range_max and msg.ranges[i] > range_min:
                    self.laser.append(msg.ranges[i])
        print self.laser    
        #print  np.rad2deg(float(msg.angle_increment)), range_max,range_min

                   

if __name__ == '__main__':
    robot = Robot()
    robot.subs()