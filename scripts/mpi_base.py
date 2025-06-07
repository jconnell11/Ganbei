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
# Copyright 2023-2025 Etaoin Systems
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
    self.hw.set_velocity(0, 0, 0)


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

    # estimate rotation from speed (sets "dr" and "wind")
    if not self.imu:
      self.dr = self.rot0 * self.dt   
      self.wind += self.dr

    # estimate travel from speed (sets "dm", "trav", "mx", and "my")
    if not self.slam:
      self.dm = self.mv0 * self.dt     
      self.trav += self.dm;
      rads = (self.wind + self.sk0) * pi / 180      
      self.mx += self.dm * cos(rads)
      self.my -= self.dm * sin(rads) 
    

  # -----------------------------------------------------------------------

  # move base with some velocity and direction
  # mgoal = cumulative travel stop, tgoal = windup angle stop 
  # moving 12 ips nominal (300 mm/sec), rate > 1 has no effect
  # turning 120 dps nominal, rate > 1 has no effect
  # skew: 0 = fwd, 90 = left (does not work well on carpet)

  def Drive(self, mgoal, tgoal, mrate, trate, skew =0, force =0):

    # possibly force odometric update
    if force > 0:
      self.Update()

    # figure new ramped speeds based on accel/decel times
    self.move = self.alter_vel(self.move, mgoal - self.trav, mrate,  12.0, 0.1, 0.1)
    self.turn = self.alter_vel(self.turn, tgoal - self.wind, trate, 120.0, 0.1, 0.1)

    # estimate actual speeds from previous cycle
    dps = self.rot0 
    if self.imu and self.dt > 0:
      dps = self.dr / self.dt
    ips = self.mv0
    if self.slam and self.dt > 0:
      ips = self.dm / self.dt

    # boost translation speed by accumulated slowness
    mv = self.move
    if mv * self.mc0 <= 0:
      self.msum = 0
    else:
      self.msum += mv - ips
      self.msum = max(-12, min(self.msum, 12))
    self.mc0 = mv
    mv += self.msum 

    # boost rotation speed by accumulated slowness
    rot = self.turn
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


  # stop all robot motion 

  def Stop(self, force =0):
    self.Drive(self.trav, self.wind, 0, 0, 0, force)


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
