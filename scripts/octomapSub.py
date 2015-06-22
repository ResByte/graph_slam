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
        self.grid_map = np.asarray([0]) #initializing empty map of size 1
        self.map_info = None
        self.map_origin = None
        self.map_res = 0.0
        self._map_pub = rospy.Publisher('OccMap', OccupancyGrid, latch=True)
        self._map_data_pub = rospy.Publisher('OccMap_metadata', 
                                             MapMetaData, latch=True)
        
    def sub(self):
        rospy.Subscriber('projected_map',OccupancyGrid, self.gridCallback,queue_size=1)
        
    def gridCallback(self,msg):
        self.grid_map = np.asarray(msg.data)
        self.map_info = msg.info
        #print self.map_info
        self.map_origin = [msg.info.origin.position.x,msg.info.origin.position.y, \
                           msg.info.origin.orientation.x,msg.info.origin.orientation.y, \
                           msg.info.origin.orientation.z,msg.info.origin.orientation.w]
        self.map_res = msg.info.resolution
        print self.grid_map.size
        print self.map_res
    def pubMap(self):
        # Publish updated map 
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        
        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.map_info.width
        grid_msg.info.height = self.map_info.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.map_origin[0], self.map_origin[1], 0),Quaternion(0, 0, 0, 1))
        
        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = self.grid.reshape((self.grid_map.size,)) * 100
        grid_msg.data = list(np.round(flat_grid))
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)
        return grid_msg
        
        
def main():
    rospy.init_node('graph_slam')
    octMap = Mapper()
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        octMap.sub()
        rate.sleep()
if __name__ == '__main__':
    main()
