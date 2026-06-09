#!/usr/bin/env python3
# encoding: utf-8

# =============================================================================
#
# mpi_grab_calib.py : fine calibration of MasterPi hand position
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# -----------------------------------------------------------------------------
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
# =============================================================================

import time, sys
import getch                           # sudo pip install getch

sys.path.append('/home/pi/Ganbei/scripts')
from mpi_hiwonder import MasterPi


# =========================================================================

# adjust arm with keyboard and save calibration values
# fine calibration near floor using shoulder (y) and elbow (z)

if __name__ == "__main__":

  # create link to expansion board
  bot = MasterPi()
  bot.LoadCal()

  # joint angles for fingertip xyz (0 7.2 1) x -40 : 0
  b0, s0, e0, w0, g0 = 0, 32.7, 45.1, 82.8, -14.9

  # initialize arm pose
  print("Posing arm ... ", end="")
  sys.stdout.flush()
  bot.Joint(3, w0, 1.0)      # wrist diagonal up 
  time.sleep(1)
  bot.Joint(4, e0, 2.0)      # elbow vertical
  time.sleep(1)
  bot.Joint(5, s0, 2.0)      # shoulder flat
  time.sleep(1)
  bot.Joint(6, b0, 2.0)      # arm forward
  time.sleep(1)
  bot.Joint(1, g0, 1.0)      # gripper closed
  time.sleep(1)
  print()

  # short instructions
  print("\nChange fingertip position using arrow keys to calibrate")
  print("Hit ENTER to save, q to quit ...\n")

  # ----------------------------------------------------------------- 
  # accept user updates of hand Y and Z values
  prompt = "Fine - gripper 89 mm (3.5\") in front and 25 mm (1\") up: "
  print("  %s 0.0 0.0 0.0 " % (prompt), end="", flush=True)
  s, e, w = s0, e0, w0
  while True:

    # check for some exit condition
    ch = getch.getch()
    if ch == 'q' or ch == 'Q':
      print("\n\nQuit!")
      sys.exit()
    if ch == '\x0A':
      print()
      break

    # alter raw cmd value (arrows prefixed by ESC)
    # changes shoulder for y, elbow for x, wrist to maintain tip
    if ch == 'A':                                
      s -= 0.5               # up
      w -= 0.5
    elif ch == 'B':                              
      s += 0.5               # down
      w += 0.5
    elif ch == 'C':                             
      e -= 0.5               # out
      w -= 0.5
    elif ch == 'D':                             
      e += 0.5               # in
      w += 0.5
    else:
      continue
    print("   \r  %s%4.1f %4.1f %4.1f " % 
          (prompt, s - s0, e - e0, w - w0), end="", flush=True) 

    # actually adjust servos (twitch)
    bot.Pose(b0, s - 5, e - 5, w + 5, g0, 0)
    time.sleep(0.1)
    bot.Pose(b0, s, e, w, g0, 0)

  # calculate equivalent new offsets
  soff, ssc = bot.GetCal(5)
  eoff, esc = bot.GetCal(4)
  woff, wsc = bot.GetCal(3)
  soff += int((s - s0) * ssc + 0.5)
  eoff += int((e - e0) * esc + 0.5)
  woff += int((w - w0) * wsc + 0.5)
  bot.SetCal(5, soff, ssc)
  bot.SetCal(4, eoff, esc)
  bot.SetCal(3, woff, wsc)
  print("    soff %d, eoff %d, woff %d\n" % (soff, eoff, woff))

  # -----------------------------------------------------------------  
  # re-establish gaze tilt equilibrium
  s0, e0, w0 = -45, 5, 5
  bot.Pose(b0, s0, e0, w0, g0, 1)

  # accept user updates of finger orientation
  prompt = "Tilt - fingers exactly horizontal: "
  print("  %s 0.0 " % (prompt), end="", flush=True)
  w = w0
  while True:

    # check for some exit condition
    ch = getch.getch()
    if ch == 'q' or ch == 'Q':
      print("\n\nQuit!")
      sys.exit()
    if ch == '\x0A':
      print()
      break

    # alter raw cmd value (arrows prefixed by ESC)
    # changes shoulder for y, elbow for x, wrist to maintain tip
    if ch == 'A' or ch == 'C':
      w += 0.5
    elif ch == 'B' or ch == 'D':  
      w -= 0.5
    else:
      continue
    print("   \r  %s%4.1f " % (prompt, w - w0), end="", flush=True) 

    # actually adjust servos (twitch)
    bot.Pose(b0, s0, e0, w + 5, g0, 0)
    time.sleep(0.1)
    bot.Pose(b0, s0, e0, w, g0, 0)

  # calculate equivalent new offset
  woff, wsc = bot.GetCal(3)
  woff += int((w - w0) * wsc + 0.5)
  bot.SetCal(3, woff, wsc)
  print("    woff %d\n" % (woff))

  # ----------------------------------------------------------------- 
  # record adjusted values in config file
  bot.SaveCal()
  print("Saved tweaked servo calibration") 
  
  