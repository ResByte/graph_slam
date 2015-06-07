#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
import numpy as np

class Mapper():
    def __init__(self):
        rospy.init_node('graph_slam')
        grid_map = np.asarray([0]) #initializing empty map of size 1
        rospy.Subscriber('projected_map',OccupancyGrid, self.gridCallback,queue_size=1)
        rospy.spin()
        
    def gridCallback(self,msg):
        self.grid_map = np.asarray(msg.data)
        print self.grid_map.size
        
def main():
    octMap = Mapper()

if __name__ == '__main__':
    main()
