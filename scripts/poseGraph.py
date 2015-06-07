#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt 
import rospy

class Node:
    def __init__(self,x = 0.0,y = 0.0,theta = 0.0,nNode= 0):
        # this is a node for pose graph
        pose_x = x
        pose_y = y
        pose_theta = theta
        nNode = nNode

class Edge:
    def __init__(self):
        weight = np.matrix([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
    def setWeight(self,wMat):
        self.weight = wMat


class PoseGraph:
    def __init__(self):
        nodeList = [] #list of nodes
        edgeList = {} #dict of edges
    def addNode(self,x,y,theta, count):
        Node node(x,y,theta,count)
        self.nodeList.append(node)
    def addEdge(self,start,end):
        #add edge to the existing 
        edgeList[start.nNode] = 
if __name__ == '__main__':
    rospy.init_node('posegraph',anonymous = True)
    rospy.spin()
