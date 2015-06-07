#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
np.set_printoptions(threshold = np.nan)


class Mapper():
    def __init__(self):
        ''' Subscribe: "projected_map" from octomap server
            Publisher: None
            Methods : gridCallback()
        '''
        grid_map = np.asarray([0]) #initializing empty map of size 1
    def sub(self):
        rospy.Subscriber('projected_map',OccupancyGrid, self.gridCallback,queue_size=1)
        
    def gridCallback(self,msg):
        self.grid_map = np.asarray(msg.data)
        print self.grid_map.size
        
def main():
    rospy.init_node('graph_slam')
    octMap = Mapper()
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        octMap.sub()
        rate.sleep()

if __name__ == '__main__':
    main()
