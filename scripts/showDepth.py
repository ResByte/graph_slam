#!/usr/bin/env python
import roslib
roslib.load_manifest('graph_slam')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.depth_sub = rospy.Subscriber("/camera/depth/image",Image,self.depth_callback)
    self.rgb_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.rgb_callback)

  def depth_callback(self,depthIm):
    try:
      depth_image = self.bridge.imgmsg_to_cv2(depthIm, "32FC1")
    except CvBridgeError, e:
      print e
    """  
    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
    """  
    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    depth_array = np.array(depth_image, dtype=np.float32)
    #cv2.normalize(depth_array, depth_array, 0, 255, cv2.NORM_MINMAX)
    for i in depth_image:
    	print i
    #print np.max(depth_array)
    #print np.min(depth_array)
    #cv2.imshow("depth", depth_array)
    #cv2.waitKey(3)

  def rgb_callback(self,rgbIm):
    try:
      rgb_image = self.bridge.imgmsg_to_cv2(rgbIm, "bgr8")
    except CvBridgeError, e:
      print e
    """  
    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
    """  
    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)
    
    cv2.imshow("rgb", rgb_image)
    cv2.waitKey(3)

    
def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)