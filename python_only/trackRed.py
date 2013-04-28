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

class colorTracking:
  def __init__(self):
     tmp = 1

  def tup2img(self, img, depth):
     a = cv.CV_8UC3
     imgOut = cv.CreateMat(depth.map.height, depth.map.width,a)
#     print "len = ", len(img)
#     array = []
#     for i in img:
#         array.append(i)
#     imrgb = Image.merge('RGB',(r,g,b))
#     imrgb = Image.fromarray(img).convert('RGB')
#     print array
#     print imrgb
     return imgOut


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
    rgb_window = "RGB"
    cv.NamedWindow(rgb_window, 0)

    try:
      while True:
        nRetVal = ctx.wait_one_update_all(depth)
#        iRetVal = rgb.get_tuple_image_map()
#        iRetVal = rgb.get_synced_image_map_bgr()
        iRetVal = rgb.get_raw_image_map_bgr()
        depthMap = depth.map
#        img = ct.tup2img(iRetVal,depth)
        img = ct.tup2img(iRetVal,depth)
#        print type(list(iRetVal))
#        img2 = Image.fromstring('BGR',(640,480),iRetVal)
        imgSize = (depthMap.width, depthMap.height)
#        img2 = Image.fromstring('L', imgSize, iRetVal, 'raw', 'F;16')


#        ppi = Image.open('/home/geovana/tig.png')

        img2 = Image.fromstring('RGB', imgSize, iRetVal)
        ppi = Image.fromstring('RGB', imgSize, iRetVal)


#        data = ppi.tostring()
        data = iRetVal
        #ci = cv.CreateImage(imgSize, cv.IPL_DEPTH_8U, 3)
        ci = cv.CreateImage(imgSize, cv.IPL_DEPTH_32F, 3)
#        cv.SetData(ci, data, len(data))
        cv.SetData(ci, data, len(data))
  
	pil_img = ppi.convert('RGB') 
        cvi = np.array(pil_img) 
        


        print "img2 = ",img2
        print "ci   = ",ci
  #      print type(img2)
#        print iRetVal[:][2]
        
        #img4= cv.LoadImage( 'tig.png' );
        #cv.ShowImage(rgb_window, img4)
        cv.ShowImage(rgb_window, cv.fromarray(cvi))
        x = depthMap.width / 2
        y = depthMap.height / 2
        cv.WaitKey(1)    
        # Get the pixel at these coordinates
        pixel = depthMap[x,y]

        print "The middle pixel is %d millimeters away." % pixel

    except KeyboardInterrupt:
      print "Shutting down"
    cv.DestroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)
