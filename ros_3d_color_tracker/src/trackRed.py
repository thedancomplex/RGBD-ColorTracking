#!/usr/bin/env python
import roslib
roslib.load_manifest('ros_3d_color_tracker')
import sys
import rospy
import cv
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

color_tracker_window = "Color Tracker"

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    cv.NamedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e

    (cols,rows) = cv.GetSize(cv_image)
    tc = self.trackColor(cv_image)
    if cols > 60 and rows > 60 :
      cv.Circle(cv_image, (tc[0],tc[1]), 20, 255)
      cv.Circle(cv_image, (200,200), 20, 255)
    cv.ShowImage("Image window", cv_image)
    cv.WaitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
      print e

  def trackColor(self,img):
            #blur the source image to reduce color noise 
            cv.Smooth(img, img, cv.CV_BLUR, 3); 
            
            #convert the image to hsv(Hue, Saturation, Value) so its  
            #easier to determine the color to track(hue) 
            hsv_img = cv.CreateImage(cv.GetSize(img), 8, 3) 
            cv.CvtColor(img, hsv_img, cv.CV_BGR2HSV) 
            
            #limit all pixels that don't match our criteria, in this case we are  
            #looking for purple but if you want you can adjust the first value in  
            #both turples which is the hue range(120,140).  OpenCV uses 0-180 as  
            #a hue range for the HSV color model 
            thresholded_img =  cv.CreateImage(cv.GetSize(hsv_img), 8, 1) 
            #cv.InRangeS(hsv_img, (120, 80, 80), (140, 255, 255), thresholded_img) 
            cv.InRangeS(hsv_img, (160, 80, 80), (180, 255, 255), thresholded_img) 
            print hsv_img[200,200]
            
            #determine the objects moments and check that the area is large  
            #enough to be our object 
            thresholded_img_mat = cv.GetMat(thresholded_img)
            moments = cv.Moments(thresholded_img_mat, 0) 
            area = cv.GetCentralMoment(moments, 0, 0) 
            
            #there can be noise in the video so ignore objects with small areas 
            x = 0
            y = 0
            if(area > 100000): 
                #determine the x and y coordinates of the center of the object 
                #we are tracking by dividing the 1, 0 and 0, 1 moments by the area 
                x = cv.GetSpatialMoment(moments, 1, 0)/area 
                y = cv.GetSpatialMoment(moments, 0, 1)/area 
            
                #print 'x: ' + str(x) + ' y: ' + str(y) + ' area: ' + str(area) 
                
                #create an overlay to mark the center of the tracked object 
                overlay = cv.CreateImage(cv.GetSize(img), 8, 3) 
                
               # cv.Circle(overlay, (x, y), 2, (255, 255, 255), 20) 
               # cv.Add(img, overlay, img) 
                #add the thresholded image back to the img so we can see what was  
                #left after it was applied 
               # cv.Merge(thresholded_img, None, None, None, img) 
            cv.ShowImage(color_tracker_window, thresholded_img)
           
            return [int(x),int(y)]  

def main(args):
  ic = image_converter()
  cv.NamedWindow( color_tracker_window, 1 ) 
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
