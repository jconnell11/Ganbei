#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# mpi_hiwonder25.py : interface to MasterPi robot using 2025 expansion board
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
#
# Copyright 2026 Etaoin Systems
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

import os, time, socket, yaml, sys
from math import radians, cos, sin

sys.path.append('/home/pi/Ganbei/Hiwonder')
from Board_25 import Board
from Sonar_2x import Sonar   


# helper function for playing standard sounds

def PlaySFX(name, wait=1, batt=0):
  cmd =  "aplay -q -D plughw:CARD=Device,DEV=0 "
  cmd += "/home/pi/Ganbei/sfx/" + name + ".wav "
  if wait == 0:
    cmd += "&"
  os.system(cmd)


# picks a low battery threshold depending on Raspberry Pi board
# Pi5 has a higher running current than Pi4 so voltage sags more
# can also use as a test for processor: LowBatt(0) > 6.5 means Pi5

def LowBatt(load=1):
  v = 7.6                    # only to 60% for 2025 expansion! 
  try:
    with open('/proc/device-tree/model', 'r') as f:
      if "Raspberry Pi 4" in f.read().strip():
        v = 6.3 
  finally:
    if load <= 0:
      v += 0.2               # rises 200mv if no load
    return v


# -------------------------------------------------------------------------


# interface to MasterPi robot using newer 2025 expansion board
# Note: built-in delay of 100 ms in Board constructor

class MasterPi:

  # initialize components and state
  def __init__(self, grab=1):

    # disconnect battery monitoring service to free up serial port
    if grab > 0:
      os.system("sudo pkill -f mpi_battery.py")
    if os.system("lsof /dev/ttyAMA0 > /dev/null") == 0:
      raise ValueError("Serial port /dev/ttyAMA0 blocked!")

    # set up to read voltage and make sure wheels are stopped
    self.bd = Board()
    self.bd.enable_reception()
    self.Freeze()     

    # connect independent sonar sensor                
    self.sn = Sonar()
    self.sn.setRGBMode(0)

    # initialize state
    self.boff, self.soff, self.eoff, self.woff, self.goff = 0, 0, 0, 0, 0
    self.bsc, self.ssc, self.esc, self.wsc, self.gsc = 11.5, 11.5, 11.5, 11.5, 13.1 
 
    # all LEDs off at beginning
    self.Body(0)
    self.Eyes(0) 


  # --------------------------- CALIBRATION -------------------------------

  # set native Hiwonder mechanism to no pulse width offet for all servos
  # only has to be called once ever (persists across power cycling)
  # will auto-retry a few times if command not initially accepted
 
  def ZeroDevs(self):
    for i in range(10):
      try:
        self.bd.pwm_servo_set_offset(1, 0)
        self.bd.pwm_servo_set_offset(3, 0)
        self.bd.pwm_servo_set_offset(4, 0)
        self.bd.pwm_servo_set_offset(5, 0)
        self.bd.pwm_servo_set_offset(6, 0)
        break
      except:
        time.sleep(0.01)         


  # load linear calibration data for servos (must call before other fcns)
  # eshews native Hiwonder mechanisms

  def LoadCal(self):
    data = {}
    cfile = "/home/pi/Ganbei/config/" + socket.gethostname() + "_servo.yaml"
    if os.path.isfile(cfile):
      with open(cfile, 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    self.bsc  = data.get('bsc', 11.5)
    self.ssc  = data.get('ssc', 11.5)
    self.esc  = data.get('esc', 11.5)
    self.wsc  = data.get('wsc', 11.7)
    self.gsc  = data.get('gsc', 13.1)
    self.boff = data.get('boff', 0)
    self.soff = data.get('soff', 0)
    self.eoff = data.get('eoff', 0)
    self.woff = data.get('woff', 0)
    self.goff = data.get('goff', 0)

 
  # return the current offset (us) and scaling (us/deg) for some servo
  # jt: 1 = gripper, 3 = wrist, 4 = elbow, 5 = shoulder, 6 = base
  
  def GetCal(self, jt):
    if jt == 1:
      return self.goff, self.gsc
    elif jt == 3:
      return self.woff, self.wsc
    elif jt == 4:
      return self.eoff, self.esc
    elif jt == 5:
      return self.soff, self.ssc
    elif jt == 6:
      return self.boff, self.bsc
    return 0, 11.7


  # update the offset (us) and scaling (us/deg) for some servo (not saved!)
  # jt: 1 = gripper, 3 = wrist, 4 = elbow, 5 = shoulder, 6 = base

  def SetCal(self, jt, off, sc):
    if jt == 1:
      self.goff, self.gsc = off, sc
    elif jt == 3:
      self.woff, self.wsc = off, sc
    elif jt == 4:
      self.eoff, self.esc = off, sc
    elif jt == 5:
      self.soff, self.ssc = off, sc
    elif jt == 6:
      self.boff, self.bsc = off, sc


  # save current servo calibration data (persists SetCal data)
  # eshews native Hiwonder mechanisms

  def SaveCal(self, backup =0):
    data = {}
    data['bsc' ] = self.bsc
    data['ssc' ] = self.ssc
    data['esc' ] = self.esc
    data['wsc' ] = self.wsc
    data['gsc' ] = self.gsc
    data['boff'] = self.boff
    data['soff'] = self.soff
    data['eoff'] = self.eoff
    data['woff'] = self.woff
    data['goff'] = self.goff  
    cfile = "/home/pi/Ganbei/config/" + socket.gethostname() + "_servo"
    with open(cfile + ".yaml", 'w') as f:
      yaml.dump(data, f, sort_keys=False)
    if backup > 0:
      os.system("cp " + cfile + ".yaml " + cfile + "_0.yaml")


  # get lo and hi angular command limits (deg) for some joint
  # jt: 1 = gripper, 3 = wrist, 4 = elbow, 5 = shoulder, 6 = base

  def Limits(self, jt, zero =0):
    off, sc = self.GetCal(jt)
    neg = (1000 - off) / sc
    pos = (1000 + off) / sc
    return zero - neg, zero + pos 


  # ----------------------------- SENSORS ---------------------------------

  # returns range (in) determined by front facing sonar

  def Prox(self):
    return self.sn.getDistance() / 25.4  


  # returns current battery voltage

  def Voltage(self):
    try:
      mv = self.bd.get_battery()
    except:
      return 0.0
    if mv is None:
      return 0.0
    return 0.001 * mv


# ---------------------------- ACTUATORS --------------------------------

  # activate or silence onboard buzzer

  def Beep(self, on, batt=0):
    if on > 0:
      self.bd.set_buzzer(523, 0.2, 0, 0)         # self-terminating (200ms)
    else:
      self.bd.set_buzzer(523, 0, 0, 1)


  # sets both backboard LEDs to some 0xRRGGBB value

  def Body(self, col):
    r =  (col >> 16)         >> 1
    g = ((col >>  8) & 0xFF) >> 1
    b =  (col        & 0xFF) >> 1
    self.bd.set_rgb([[1, r, g, b], [2, r, g, b]])


  # sets both sonar LEDs to some 0xRRGGBB value

  def Eyes(self, col):
    r = col >> 16;
    g = (col >> 8) & 0xFF
    b = col & 0xFF
    self.sn.setPixelColor(0, (r, g, b))
    self.sn.setPixelColor(1, (r, g, b))  
    self.sn.show()           


  # set an individual servo command (usec) and transition time (sec)
  # jt: 1 = gripper, 3 = wrist, 4 = elbow, 5 = shoulder, 6 = base
  # will auto-retry a few times if command not initially accepted

  def Servo(self, id, wid, ramp):
    for i in range(10):
      try:
        self.bd.pwm_servo_set_position(ramp, [[id, int(wid)]])
        break
      except:
        time.sleep(0.01)


  # set an individual servo angle (deg) and transition time (sec)
  # jt: 1 = gripper, 3 = wrist, 4 = elbow, 5 = shoulder, 6 = base
  # will auto-retry a few times if command not initially accepted

  def Joint(self, jt, ang, ramp):
    off, sc = self.GetCal(jt)
    self.Servo(jt, 1500 + int(sc * ang + 0.5) + off, ramp)


  # command full set of arm angles (deg) and transition time (sec)
  # must call LoadCal at beginning to get accurate positioning

  def Pose(self, b, s, e, w, g, ramp):
    bw = 1500 + int(self.bsc * b + 0.5) + self.boff
    sw = 1500 + int(self.ssc * s + 0.5) + self.soff
    ew = 1500 + int(self.esc * e + 0.5) + self.eoff
    ww = 1500 + int(self.wsc * w + 0.5) + self.woff
    gw = 1500 + int(self.gsc * g + 0.5) + self.goff
    self.bd.pwm_servo_set_position(ramp, [[6, bw], [5, sw], [4, ew], [3, ww], [1, gw]])


  # stop all wheels immediately
  # will auto-retry a few times if command not initially accepted

  def Freeze(self):
    for i in range(10):
      try:
        self.bd.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])
        break
      except:
        time.sleep(0.01)


  # sets wheel commands based on move speed, driving angle, and turn speed
  # mv is ips (12 max), skew is ccw degs from forward, rot is ccw dps (120 max) 
  # returns factor by which speeds were slowed down due to saturation, 0 if problem

  def Mecanum(self, mv, skew, rot):

    # convert to correct units (empirical calibration)
    veer = radians(skew + 90)
    move = 4.0 * mv
    turn = radians(-0.135 * rot)

    # modified from Hiwonder mecanum.py
    #    motor1 v1|  ↑  |v2 motor2
    #             |     |
    #    motor3 v3|     |v4 motor4
    a = 67                             # wheel mm left/right
    b = 59                             # wheel mm front/back 
    vp = -turn * (a + b)
    vx = move * cos(veer)
    vy = move * sin(veer)
    v1 = int(vy + vx - vp) 
    v2 = int(vy - vx + vp)
    v3 = int(vy - vx - vp)
    v4 = int(vy + vx + vp)

    # motors max out at +/- 100 so make sure to maintain ratios
    rein = 1.0
    top = max(abs(v1), abs(v2), abs(v3), abs(v4)) / 100
    if top > 1:
      rein = 1 / top
      v1 = int(rein * v1)
      v2 = int(rein * v2)
      v3 = int(rein * v3)
      v4 = int(rein * v4)

    # set motor duty cycles 
    try:
      self.bd.set_motor_duty([[1, -v1], [2, v2], [3, -v3], [4, v4]])
      return rein
    except:
      return 0


# =========================================================================

# simple test of various functionalities

if __name__ == "__main__":
  bot = MasterPi()

  # turn eyes dark purple and return some range values
  print("Purple eyes")
  bot.Eyes(0x8000FF)
  bot.Body(0x00FF00)
  for i in range(10):
    print("  sonar = %3.1f\"" % (bot.Prox()))
    time.sleep(0.2)
  print("Green eyes")
  bot.Eyes(0x00FF00)
  bot.Body(0x8000FF)
#  PlaySFX("toot")
  time.sleep(1)

  """
  # configure arm servos
  print("Arm Pose ...")  
  bot.LoadCal()
  bot.Joints(15, -40, 0, -25, 10, 2)
  time.sleep(2.0)
  
  # drive diagonal, turn 90, then backup
  print("Diagonal @ 45 ...")
  bot.Mecanum(12, 45, 0)
  time.sleep(2.0)
  print("Turn Right ...")
  bot.Mecanum(0, 0, -90)
  time.sleep(2.0)
  print("Backwards ...")
  bot.Mecanum(-8, 0, 0)
  time.sleep(2.0)
  bot.Freeze()
  """

  # report battery level
  bot.Eyes(0)
  bot.Body(0)
  print("Battery = %4.2fv" % (bot.Voltage()))
