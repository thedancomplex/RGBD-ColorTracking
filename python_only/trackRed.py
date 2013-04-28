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

  def raw2cvImg(self, imgRaw, depth):
     imgSize = (depth.map.width, depth.map.height)
     ppi = Image.fromstring('RGB', imgSize, imgRaw)
     pil_img = ppi.convert('RGB') 
     cvi = np.array(pil_img) 
     outImg = cv.fromarray(cvi)
     return outImg


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
        iRetVal = rgb.get_synced_image_map_bgr()
#        iRetVal = rgb.get_raw_image_map_bgr()
        depthMap = depth.map

        cvi = ct.raw2cvImg(iRetVal,depth)
        print "cvi   = ",cvi
        cv.ShowImage(rgb_window, cvi)
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
