#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# mpi_imu.py : gets data from Adafruit BNO085 accelerometer board
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
#
# Copyright 2024-2025 Etaoin Systems
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

import serial, math, time
from adafruit_bno08x import (BNO_REPORT_GAME_ROTATION_VECTOR)
from adafruit_bno08x.uart import BNO08X_UART


# gets data from Adafruit BNO085 accelerometer board
# should wait about 1 sec after start for data to stabilize
# Note: need to run bno08x_calib.py when first installed

class MpiImu:

  # create receiver component and set parameters
  def __init__(self):

    # connect board in UART mode (PS1 = hi, TX->SCL, RX<-SDA)
    uart = serial.Serial("/dev/ttyAMA0", 3000000)
    try:
      self.bno = BNO08X_UART(uart)     # first call often fails!
    except:
      self.bno = BNO08X_UART(uart)
    self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)  

    # orientation never sampled yet, no tilt or roll offsets
    self.init = 0
    self.tsum = 0.0
    self.rsum = 0.0
    self.t0 = 0.0
    self.r0 = 0.0

    # cached output value
    self.yaw   = 0.0
    self.pitch = 0.0
    self.roll  = 0.0


  # sample BNO085 data stream and resolve into components
  # Note: should wait about 1 sec before calling for first time

  def Update(self):

    # variable "quaternion" maintained by "bno" component 
    # mounted so that +y is forward and +x is right 
    qx, qy, qz, qw = self.bno.game_quaternion

    # conversion from Wikipedia (roll CCW)
    ysamp = math.atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
    psamp = math.atan2(2*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
    rsamp = math.asin( 2*(qx*qz - qw*qy))
    self.yaw   = 180 * ysamp / math.pi   
    self.pitch = 180 * psamp / math.pi - self.t0
    self.roll  = 180 * rsamp / math.pi - self.r0  

    # estimate tilt and roll offsets during first 100 samples
    if self.init < 100:
      self.tsum += self.pitch
      self.rsum += self.roll
      self.init += 1
      if self.init == 100:
        self.t0 = 0.01 * self.tsum
        self.r0 = 0.01 * self.rsum


  # get body planar orientation  
  # returns CCW degrees wrt magnetic north (= 0) 

  def Heading(self, force =0):
    if force > 0 or self.init <= 0:
      self.Update()
    return self.yaw


  # get body incline wrt gravity
  # returns degrees (pos = front up, 0 = flat, neg = front down)

  def Incline(self, force =0):
    if force > 0 or self.init <= 0:
      self.Update()
    return self.pitch


  # get body roll around centerline
  # returns degrees (pos = right higher, 0 = flat, neg = left higher)

  def Roll(self, force =0):
    if force > 0 or self.init <= 0:
      self.Update()
    return self.roll


# =========================================================================

# report orientation for 20 seconds

if __name__ == "__main__":
  imu = MpiImu()
  print("20 secs of live orientation ...")
  print("\033[?25l", end="")           # suppress cursor
  for i in range(600):
    imu.Update()
    h = imu.Heading()
    t = imu.Incline()
    r = imu.Roll()
    print("  %4.0f = H, %3.0f = T, %3.0f = R" % (h, t, r), end ="\r")
    time.sleep(0.033)
  print("\ndone")
  print("\033[?25h", end="")           # restore cursor
 

