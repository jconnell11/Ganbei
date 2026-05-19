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
# Copyright 2023-2026 Etaoin Systems
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

import time
from math import radians, cos, sin, sqrt                       

from mpi_hiwonder import MasterPi      # for testing 


# -------------------------------------------------------------------------

# interface to MasterPi robot drive wheels and odometry

class MpiBase:

  # initialize state (takes robot interface object as argument)
  def __init__(self, mpi):
    self.bot = mpi

    # ramped move and turn velocities
    self.move = 0.0
    self.turn = 0.0

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

    # cumulative motion, map location, and source of info
    self.trav = 0.0
    self.wind = 0.0
    self.mx   = 0.0
    self.my   = 0.0
    self.imu  = False
    self.slam = False

    # make sure robot is not moving
    self.bot.Freeze()


  # attempt clean shutdown
  def __del__(self):
    self.bot.Freeze()


  # -----------------------------------------------------------------------

  # report travel, windup, and estimated map position of robot
  # heading of 0 points along x axis, 90 points along y axis

  def Odom(self):
    return self.trav, self.wind, self.mx, self.my


  # directly set the robot's map heading (e.g. from IMU)
  # (magnetic) compass angle is generally not the same as map angle
  # map heading of 0 points along x axis, 90 points along y axis
  # sets values "dr" and "wind" (and "hd0")

  def Compass(self, ccw):
    if not self.imu:
      self.dr = 0.0                    # no "ccw0" first call
    else:
      self.dr = ccw - self.ccw0        
      if self.dr > 180:
        self.dr -= 360
      elif self.dr <= -180:
        self.dr += 360    
    self.wind += self.dr;              # cumulative turn
    self.ccw0 = ccw                     
    self.imu = True                    # never resets
   

  # directly set the robot's map position (e.g. from SLAM)
  # heading of 0 points along x axis, 90 points along y axis
  # sets values "dm", "trav", "mx", and "my"

  def Map(self, xmid, ymid):
    dx = xmid - self.mx 
    dy = ymid - self.my 
    self.dm = sqrt(dx * dx + dy * dy) 
    if self.mv0 < 0.0:
      self.dm = -self.dm
    self.trav += self.dm;              # cumulative travel
    self.mx = xmid                     
    self.my = ymid
    self.slam = True                   # never resets


  # possibly adjust current odometry estimates using wheel speeds

  def Update(self):

    # get elapsed time since last command 
    last = self.tcmd
    self.tcmd = time.time()
    self.dt = 0
    if last > 0:
      self.dt = self.tcmd - last       

    # estimate rotation from speed
    if not self.imu:
      self.dr = 0.7 * self.rot0 * self.dt      # scrub compensation
      self.wind += self.dr

    # estimate travel from speed 
    if not self.slam:
      self.dm = self.mv0 * self.dt  
      self.trav += self.dm;
      rads = radians(self.wind + self.sk0)     
      self.mx += self.dm * cos(rads)
      self.my += self.dm * sin(rads) 
 

  # -----------------------------------------------------------------------

  # move base with some velocity and direction
  # mgoal = cumulative travel stop, tgoal = windup angle stop 
  # moving 12 ips nominal (300 mm/sec), turning 120 dps nominal
  # skew: 0 = fwd, 90 = left (does not work well on carpet)

  def Drive(self, mgoal, tgoal, mrate, trate, skew =0, force =0):

    # possibly force odometric update
    if force > 0:
      self.Update()
    
    # figure new ramped speeds based on accel/decel times 
    # canonical rotation is 180 dps (but only effectively 120 dps)
    self.move = self.alter_vel(self.move, mgoal - self.trav, mrate,  12.0, 0.1, 0.1)
    self.turn = self.alter_vel(self.turn, tgoal - self.wind, trate, 180.0, 0.1, 0.2)

    # estimate actual speeds from previous cycle
    dps = self.rot0 
    if self.imu and self.dt > 0:
      dps = self.dr / self.dt
    ips = self.mv0
    if self.slam and self.dt > 0:
      ips = self.dm / self.dt

    # boost translation speed by accumulated slowness (for SLAM mostly)
    mv = self.move
    if mv * self.mc0 <= 0:
      self.msum = 0
    else:
      self.msum += mv - ips
      self.msum = max(-12, min(self.msum, 12))
    self.mc0 = mv
    mv += self.msum 

    # boost rotation speed by accumulated slowness (for IMU mostly)
    rot = self.turn
    if rot * self.rc0 <= 0:
      self.rsum = 0
    else:
      self.rsum += rot - dps
      self.rsum = max(-180, min(self.rsum, 180))
    self.rc0 = rot
    rot += self.rsum  

    # clamp command speeds to effective motion range
    mv = max(-12, min(mv, 12))
    if abs(mv) < 6:
      mv = 0 
    rot = max(-180, min(rot, 180))
    if abs(rot) < 90:
      rot = 0 

    # send to wheel motors (does argument conversions)
    rein = self.bot.Mecanum(mv, skew, rot)

    # cache current effective command speeds for next cycle
    if rein > 0:
      self.mv0  = rein * mv
      self.rot0 = rein * rot
      self.sk0  = skew


  # change velocity at rate "rt" to reduce "inc" remaining change
  # scales accelerations to give same trajectory regardless of rate
  # makes sure that limited deceleration will cause stop at goal
  # inc is distance from goal, v is current speed, vn is nominal speed
  # rt is speed multiplier, tup and tdn are accel/decel seconds 
  #
  #        ^
  #     sp |       +-----------
  #        |     /
  #        |   /
  #        | /
  #       -+------------------->
  #                       dist

  def alter_vel(self, v, inc, rt, vn, tup, tdn):
 
    # get accelerations from ramp times 
    vmax = rt * vn
    acc = rt * vmax / max(tup, 0.01)
    dec = rt * vmax / max(tdn, 0.01)

    # changes in speed are relative to goal direction
    if inc < 0.0:
      acc = -acc
      dec = -dec

    # if going wrong way decelerate toward zero
    if inc * v < 0.0:
      v2 = v + dec * self.dt
    else:
      # accelerate (assuming far from goal)
      v2 = v + acc * self.dt 

      # limit speed by goal deceleration
      vstop = sqrt(2.0 * dec * inc)
      if vstop < vmax:
        vmax = vstop

    # clip speed to valid range
    if v2 > vmax:
      v2 = vmax
    elif v2 < -vmax:
      v2 = -vmax
    return v2


  # stop all robot motion and possibly update odometry

  def Stop(self, force =0):
    self.Drive(self.trav, self.wind, 0, 0, 0, force)


# =========================================================================

# simple test tries translation, rotation, and skew driving

if __name__ == "__main__":
  bot = MasterPi()
  b = MpiBase(bot)
  b.Update()
  t, r, x, y = b.Odom()

  
  # fast motion test
  print("forward 18 inches")
  t0, r0, x0, y0 = t, r, x, y
  for i in range(40):  
    b.Drive(t0 + 18, r0, 1, 0)             
    time.sleep(0.05)
    b.Update()
  b.Stop()
  t, r, x, y = b.Odom()
  print("  -> move: (%4.2f %4.2f), turn: %3.1f" % (x - x0, y - y0, r - r0))

  print("left 180 degrees (CW)")
  t0, r0, x0, y0 = t, r, x, y
  for i in range(40):   
    b.Drive(t0, r0 + 180, 0, 1)             
    time.sleep(0.05)
    b.Update()
  b.Stop()
  t, r, x, y = b.Odom()
  print("  -> move: (%4.2f %4.2f), turn: %3.1f" % (x - x0, y - y0, r - r0))
  
  print("diagonal right 18 inches (dx,dy = 12.7)")
  t0, r0, x0, y0 = t, r, x, y
  for i in range(40):
    b.Drive(t0 + 18, r0, 1, 0, -45)               
    time.sleep(0.05)
    b.Update()
  b.Stop()
  t, r, x, y = b.Odom()
  print("  -> move: (%4.2f %4.2f), turn: %3.1f" % (x - x0, y - y0, r - r0))
  

  """
  # slow motion test (better with Compass)
  print("turn CCW 180 degs at 15 dps")
  t0, r0, x0, y0 = t, r, x, y
  for i in range(300):
    b.Drive(t0, 0, r0 + 180, 0.125)
    time.sleep(0.05)
    b.Update()
  b.Stop()
  t, r, x, y = b.Odom()
  print("  -> turn: %3.1f" % (r0 - r))
  """

  """
  # turn calibration for speed and scrub factors
  sp = 1.0
  for j in range(40):
    b.Drive(0, 360, 0, sp)
    time.sleep(0.05)
    b.Update()
    t, r, x, y = b.Odom()
    print(" head %4.2f" % (r))
  print("speed %4.2f ------------" % (sp))
  bot.Freeze()
  """
