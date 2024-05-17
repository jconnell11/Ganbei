#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# mpi_arm_cal.py : simple utility to fix angles of MasterPi 4 DOF arm
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
#
# Copyright 2024 Etaoin Systems
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

import time, yaml, sys
import getch             # sudo pip3 install getch

sys.path.append('/home/pi/MasterPi/HiwonderSDK')
import Board


# move the robot to pre-calibration pose then canonical 
# default arm straight, later phases modify elbow and wrist bend

def test_dev(sid, de =0, dw =0, t0 =0):

  # precalibration pose (so it will twitch)
  t = 1500 if sid == 0 else t0
  ep = int(11.3636 * de + 0.5)
  wp = int(11.3636 * dw + 0.5)
  if sid == 0 or sid == 6:
    Board.setPWMServoPulse(6, 1550, t)         # base
  if sid == 0 or sid == 5:
    Board.setPWMServoPulse(5, 2050, t)         # shoulder
  if sid == 0 or sid == 4:
    Board.setPWMServoPulse(4, 1050 + ep, t)    # elbow
  if sid == 0 or sid == 3:
    Board.setPWMServoPulse(3, 1450 + wp, t)    # wrist
  if sid == 0 or sid == 1:
    Board.setPWMServoPulse(1, 1550, t)         # gripper
  time.sleep(0.2 + 0.001 * t)

  # calibration configuration
  if sid == 0 or sid == 6:
    Board.setPWMServoPulse(6, a2p(0),     0)   # base
  if sid == 0 or sid == 5:
    Board.setPWMServoPulse(5, a2p(45),    0)   # shoulder 
  if sid == 0 or sid == 4:
    Board.setPWMServoPulse(4, a2p(de-45), 0)   # elbow 
  if sid == 0 or sid == 3:
    Board.setPWMServoPulse(3, a2p(dw),    0)   # wrist 
  if sid == 0 or sid == 1:
    Board.setPWMServoPulse(1, a2p(0),     0)   # gripper
  time.sleep(0.2)


# get servo pulse length before correction
# empirically 800-2200 -> 123 degs, so 176 deg full swing

def a2p(ang):
  return int(11.3636 * ang + 1500 + 0.5)


# change some joint deviation using keyboard
# negative sid reverses meaning of arrow keys
# stops whole program if 'q', exits if 'enter'

def keyb_alt(servo, data, de =0, dw =0):

  # set up target ID and direction
  inc = 5 if servo >= 0 else -5
  sid = abs(servo)
  if de != 0 or dw != 0:
    test_dev(sid, de, dw, 500)
  print("  jt", sid, "=", data[str(sid)], "   ", end = "\r")
  while True:

    # loop until some exit condition
    ch = getch.getch()
    if ch == 'q' or ch == 'Q':
      print("\n")
      sys.exit()
    if ch == '\x0A':
      return
    
    # look for some arrow key (has "ESC[" prefix)
    if ch != '\x1B':
      continue
    if getch.getch() != '[':
      continue
    ch = getch.getch()

    # alter relevant deviation value
    if ch == 'A' or ch == 'C':
      data[str(sid)] += inc
    elif ch == 'B' or ch == 'D':  
      data[str(sid)] -= inc
    else:
      continue
    print("  jt", sid, "=", data[str(sid)], "   ", end = "\r")
  
    # save changed value to file then move arm to check again 
    try:
      with open("/home/pi/MasterPi/Deviation.yaml", 'w') as f:
        yaml.dump(data, f)
    except:
      print("\nCould not write calibration file!")
      sys.exit()
    test_dev(sid, de, dw)  
 

# =========================================================================

# adjust arm with keyboard and save calibration values

if __name__ == "__main__":

  # read current servo deviations (if any)
  print('Getting ready to alter file ~/MasterPi/Deviation.yaml ...')
  try:
    with open("/home/pi/MasterPi/Deviation.yaml", 'r') as f:
      data = yaml.load(f, Loader=yaml.FullLoader)
  except:
    data = {'1':0, '3':0, '4':0, '5':0, '6':0}

  # initialize arm pose
  test_dev(0)
  print('Change joint angles using arrow keys to calibrate')
  print('Hit ENTER to go to next step, q to quit')

  # adjust joints saving values at each step
  print('\nBase: make arm point along centerline of robot body ...')
  keyb_alt(-6, data)
  print('\nGripper: adjust so finger opening under stress is 23mm ...')
  keyb_alt(1, data)
  print('\nElbow: move axis onto straight edge from shoulder to wrist ...')
  keyb_alt(-4, data)
  print('\nWrist: align gripper jaws with straightened arm ...')
  keyb_alt(3, data)
  
  # improve accuracy of kinematics
  print('\n\nShoulder: elevate so finger pads are 276mm above surface ...')
  keyb_alt(-5, data)
  print('\nElbow (again): bend so finger pads are 170mm above surface ...')
  keyb_alt(-4, data, 45)
  print('\nWrist (again): bend so finger pads are 108mm above surface ...')
  keyb_alt(3, data, 45, -45)
  print('\nAll steps completed')
