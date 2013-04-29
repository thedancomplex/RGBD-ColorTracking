#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */


import sys
import cv
import cv2
import numpy as np
from openni import *

from PIL import Image
from numpy import eye 


color_tracker_window = "Color Tracking Window"
rgb_window = "RGB Window"




class colorTracking:
  def __init__(self):
     tmp = 1

  def raw2cvImg(self, imgRaw, depth):
     imgSize = (depth.map.width, depth.map.height)
     ppi = Image.fromstring('RGB', imgSize, imgRaw)
     pil_img = ppi.convert('RGB') 
     cvi = np.array(pil_img) 
     outImg = cv.fromarray(cvi)
     return outImg


  def trackColor(self,img):
            #blur the source image to reduce color noise 
            cv.Smooth(img, img, cv.CV_BLUR, 5);

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
            ## Red
            #cv.InRangeS(hsv_img, (160, 80, 100), (180, 255, 255), thresholded_img) 
            ## Better red?
            #cv.InRangeS(hsv_img, (0, 80, 100), (20, 255, 255), thresholded_img) 

            ## Yellow
            cv.InRangeS(hsv_img, (25, 80, 100), (40, 255, 255), thresholded_img)
  #          print hsv_img[200,200]

            #determine the objects moments and check that the area is large  
            #enough to be our object 
            thresholded_img_mat = cv.GetMat(thresholded_img)
            moments = cv.Moments(thresholded_img_mat, 0)
            area = cv.GetCentralMoment(moments, 0, 0)


            #there can be noise in the video so ignore objects with small areas 
            x = -1;
            y = -1;
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
    ct = colorTracking()
    ctx = Context()
    ctx.init()

    # Create a depth generator
    depth = DepthGenerator()
    rgb = ImageGenerator()
  #  rgb = ctx.find_existing_node(NODE_TYPE_IMAGE)
    depth.create(ctx)
    rgb.create(ctx)

    # Set it to VGA maps at 30 FPS
    depth.set_resolution_preset(RES_VGA)
    rgb.set_resolution_preset(RES_VGA)
    depth.fps = 30
    rgb.fps = 30

    # Start generating
    ctx.start_generating_all()
    cv.NamedWindow(rgb_window, 0)
    cv.NamedWindow(color_tracker_window, 0)

    try:
      while True:
        nRetVal = ctx.wait_one_update_all(depth)
#        iRetVal = rgb.get_tuple_image_map()
        iRetVal = rgb.get_synced_image_map_bgr()
#        iRetVal = rgb.get_raw_image_map_bgr()
        depthMap = depth.map

        cvi = ct.raw2cvImg(iRetVal,depth)

        # Track the Color
        (x,y) = ct.trackColor(cvi)




        cv.ShowImage(rgb_window, cvi)
#        x = depthMap.width / 2
#        y = depthMap.height / 2
        cv.WaitKey(1)    
        # Get the pixel at these coordinates
        if (x >= 0):
            pixel = depthMap[x,y]
            print "The tracked pixel is %d millimeters away." % pixel
        else:
            print "No Object"

    except KeyboardInterrupt:
      print "Shutting down"
    cv.DestroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)
