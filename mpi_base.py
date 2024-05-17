#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# mpi_base.py : interface to MasterPi robot drive wheels and odometry
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
#
# Copyright 2023-2024 Etaoin Systems
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

import time, sys
from math import pi, cos, sin, sqrt                       

sys.path.append('/home/pi/MasterPi/HiwonderSDK')
from mecanum import MecanumChassis


# interface to MasterPi robot drive wheels and odometry
# Mecanum: a = half forward-back dist, b = half left-right dist 

class MpiBase:

  # set parameters and initialize state

  def __init__(self):
    self.hw = MecanumChassis()
    self.hw.set_velocity(0, 0, 0)

    # prior command speeds and integral error terms
    self.mc0  = 0.0
    self.rc0  = 0.0
    self.msum = 0.0
    self.rsum = 0.0

    # modified form of last command sent to wheels
    self.mv0  = 0.0
    self.rot0 = 0.0
    self.sk0  = 0.0

    # time since last update and incremental movement
    self.tcmd = 0.0
    self.dt   = 0.0
    self.dr   = 0.0
    self.dm   = 0.0

    # map orientation, location, and source of info
    self.mh   = 0.0
    self.mx   = 0.0
    self.my   = 0.0
    self.imu  = False
    self.slam = False

    # make sure robot is not moving
    self.Stop(1)


  # -----------------------------------------------------------------------

  # report estimated map position and heading of robot
  # heading of 0 points along x axis, 90 points along y axis

  def Odom(self):
    return self.mx, self.my, self.mh


  # directly set the robot's map heading (e.g. from IMU)
  # heading of 0 points along x axis, 90 points along y axis

  def Compass(self, ccw):
    self.dr = ccw - self.mh            # for speed feedback
    if self.dr > 180:
      self.dr -= 360
    elif self.dr <= -180:
      self.dr += 360    
    self.mh = ccw
    self.imu = True                    # never resets
   

  # directly set the robot's map position (e.g. from SLAM)
  # heading of 0 points along x axis, 90 points along y axis

  def Map(self, xmid, ymid):
    dx = xmid - self.mx 
    dy = ymid - self.my 
    self.dm = sqrt(dx * dx + dy * dy)  # for speed feedback
    self.mx = xmid
    self.my = ymid
    self.slam = True                   # never resets


  # possibly adjust current odometry estimates using wheel speeds

  def Update(self):

    # get elapsed time since last command (for speed feedback)
    last = self.tcmd
    self.tcmd = time.time()
    self.dt = 0
    if last > 0:
      self.dt = self.tcmd - last       # for speed feedback

    # estimate travel and rotation from speeds (if needed)
    if not self.imu:
      self.dr = self.rot0 * self.dt    # for speed feedback
      self.mh += self.dr
      if self.mh >= 360:
        self.mh -= 360
      elif self.mh < 0:
        self.mh += 360
    if not self.slam:
      self.dm = self.mv0 * self.dt     # for speed feedback
      rads = self.sk0 * pi / 180       # requested direction
      self.mx += self.dm * cos(rads)
      self.my -= self.dm * sin(rads) 
    

  # -----------------------------------------------------------------------

  # move base with some velocity and direction
  # move = 12 ips nominal (300 mm/sec), rate > 1 has no effect
  # turn = 120 dps nominal, rate > 1 has no effect
  # skew: 0 = fwd, 90 = left (does not work well on carpet)

  def Drive(self, move, turn, skew =0, force =0):

    # possibly force odometric update
    if force > 0:
      self.Update()

    # estimate actual speeds from previous cycle
    dps = self.rot0 
    if self.imu and self.dt > 0:
      dps = self.dr / self.dt
    ips = self.mv0
    if self.slam and self.dt > 0:
      ips = self.dm / self.dt

    # boost translation speed by accumulated slowness
    mv = 12.0 * move
    if mv * self.mc0 <= 0:
      self.msum = 0
    else:
      self.msum += mv - ips
      self.msum = max(-12, min(self.msum, 12))
    self.mc0 = mv
    mv += self.msum 

    # boost rotation speed by accumulated slowness
    rot = 120.0 * turn
    if rot * self.rc0 <= 0:
      self.rsum = 0
    else:
      self.rsum += rot - dps
      self.rsum = max(-120, min(self.rsum, 120))
    self.rc0 = rot
    rot += self.rsum  

    # clamp command speeds to effective motion range
    mv = max(-12, min(mv, 12))
    if abs(mv) < 6:
      mv = 0 
    rot = max(-120, min(rot, 120))
    if abs(rot) < 90:
      rot = 0 
    if mv == self.mv0 and rot == self.rot0 and skew == self.sk0:
      return

    # convert to correct units and send to wheel motors
    veer = skew + 90
    move = 6.1 * mv
    turn = -0.2 * rot * pi / 180
    self.hw.set_velocity(move, veer, turn)

    # cache current effective command speeds for next cycle
    self.mv0  = mv
    self.rot0 = rot
    self.sk0  = skew


  # stop all robot motion 

  def Stop(self, force =0):
    self.Drive(0, 0, 0, force)


# =========================================================================

# simple test tries translation, rotation, and skew driving

if __name__ == "__main__":
  b = MpiBase()
  b.Update()
  
  
  # fast motion test
  print("forward 18 inches")
  b.Drive(1.0, 0, 0)
  time.sleep(1.5)
  b.Update()
  b.Stop()
  y, x, r = b.Odom()
  print("  -> move: (%4.2f %4.2f), turn: %3.1f" % (y, x, r))
  
  print("left 180 degrees (CW)")
  b.Drive(0, 1.0, 0)                
  time.sleep(1.5)
  b.Update()
  b.Stop()
  y, x, r = b.Odom()
  print("  -> move: (%4.2f %4.2f), turn: %3.1f" % (y, x, r))
  
  print("diagonal right 18 inches (dy,dx = 12.7)")
  b.Drive(1.0, 0, -45)
  time.sleep(1.5)
  b.Update()
  b.Stop()
  y, x, r = b.Odom()
  print("  -> move: (%4.2f %4.2f), turn: %3.1f" % (y, x, r))


  """
  # slow motion test (better with Compass)
  print("turn CCW 180 degs at 15 dps")
  rot = 0.0
  for i in range(360):
    b.Drive(0, 0.125, 0)
    time.sleep(0.0322)                 # loop takes 1.1ms
    b.Update()
    _,_,r = b.Odom()
    rot += r
  b.Stop()
  print("  -> turn: %3.1f" % (rot))
  """
