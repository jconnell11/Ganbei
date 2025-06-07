#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# mpi_cam.py : Python wrapper for OpenCV interface to SM371 USB camera 
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
#
# Copyright 2025 Etaoin Systems
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# =========================================================================

import numpy as np, cv2, sys, time
from ctypes import CDLL, POINTER, cast, c_ubyte, c_void_p

# bind shared library     
lib = CDLL('lib/libmpi_cam.so')            

# define return types of image function (buffer pointers)
lib.cam_color.restype  = c_void_p


# Python wrapper for OpenCV interface to SM371 USB camera (MasterPi)

class MpiCam:

  # connect to default VGA format camera
  # returns 1 if okay, 0 or negative for problem

  def Start(self):
    return lib.cam_start()


  # get VGA color image, possibly waiting for new frame (block = 1)
  # image fmt: 0 = buffer pointer (int), 1 = OpenCV Mat (numpy ndarray)
  # returns pointer to image or None if not ready or broken

  def Color(self, block =0, fmt =1):
    ptr = lib.cam_color(block)
    if not ptr:
      return None
    if fmt <= 0:
      return ptr             # int = memory address
    buf = cast(ptr, POINTER(c_ubyte * 480 * 640 * 3))
    img = np.frombuffer(buf.contents, np.uint8)
    img.shape = (480, 640, 3)
    return img


  # cleanly disconnect camera

  def Done(self):
    lib.cam_done()


# =========================================================================

# simple test program

if __name__ == "__main__":       

  show = 1                             # show images
  if len(sys.argv) > 1:
    if sys.argv[1].isdigit():
      show = int(sys.argv[1])
    else:
      print("argument = show images (1) or not (0)")      

  # connect to sensor and make display window
  rgb = MpiCam()  
  if rgb.Start() <= 0:
    print("Could not connect to color camera!")
    sys.exit(0)
  if show > 0:
    cv2.namedWindow("Camera")
    cv2.moveWindow("Camera", 10, 10)

  # receive frames and display them
  print("Streaming video ...")
  i = 0
  start = time.time();
  t0 = start
  ft = start
  try:
    while True:
      img = rgb.Color(0)
      if img is None:
        time.sleep(0.001)
        continue
      i += 1

      now = time.time()
      ms = 1000 * (now - ft)
      ft = now
      if ms > 200.0:
        print("-- frame %5.1f ms" % (ms))
      if (i % 30) == 0:
        print("%3.1f fps" % (30.0 / (now - t0)))
        t0 = now 

      if show > 0:
        cv2.imshow("Camera", img) 
        cv2.waitKey(1)                 # pump update message
  except KeyboardInterrupt:
    print("")

  # shutdown after error or Ctrl-C
  stop = time.time();
  rgb.Done()
  if show > 0:
    cv2.destroyAllWindows() 
  print("Camera stopped - %d frames at %4.2f fps" % (i, i / (stop - start)))
