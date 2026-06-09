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
# Copyright 2025-2026 Etaoin Systems
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
import getch                           # sudo pip install getch

sys.path.append('/home/pi/Ganbei/scripts')
from mpi_hiwonder import MasterPi


# -------------------------------------------------------------------------

# change some joint pulse width using arrow keys 
# stops whole program if 'q', exits if 'enter'

def solicit(jt, ang, prompt):
  global bot

  # set servo to initial angle
  off, sc = bot.GetCal(jt)
  pw = 1500 + int(sc * ang + 0.5) + off
  bot.Servo(jt, pw, 1)
  time.sleep(1)

  # accept user updates of pulse width
  print("  %s%4d " % (prompt, pw), end="")
  sys.stdout.flush()
  while True:

    # check for some exit condition
    ch = getch.getch()
    if ch == 'q' or ch == 'Q':
      print("\n\nQuit!")
      sys.exit()
    if ch == '\x0A':
      break

    # alter deviation value (arrows prefixed by ESC)
    if ch == 'A' or ch == 'C':
      pw += 5
    elif ch == 'B' or ch == 'D':  
      pw -= 5
    else:
      continue
    print("          \r  %s%4d " % (prompt, pw), end="")
    sys.stdout.flush()

    # actually adjust servo (twitch)
    bot.Servo(jt, pw - 50, 0)
    time.sleep(0.2)
    bot.Servo(jt, pw, 0)

  # end prompt line and give best value
  print()
  return pw


# determine zero offset width and scaling for servo based on lo and hi
# assumes "arc" (deg) is symmetric around servo neutral (1500us) position

def swing(lo, hi, arc =90): 
  off = int(0.5 * (hi + lo) + 0.5) - 1500
  sc  = (hi - lo) / arc
  return off, round(sc, 2)


# =========================================================================

# adjust arm with keyboard and save calibration values

if __name__ == "__main__":
  global bot

  # create link to expansion board
  bot = MasterPi()
  bot.LoadCal()

  # initialize arm pose
  print("Posing arm ... ", end="")
  sys.stdout.flush()
  bot.Joint(4, -45, 1)       # elbow inline
  time.sleep(1)
  bot.Joint(5, 0, 1)         # shoulder diagonal up
  time.sleep(1)
  bot.Joint(3, -45, 1)       # wrist inline
  time.sleep(1)
  bot.Joint(6, 0, 1)         # arm forward
  time.sleep(1)
  bot.Joint(1, 14.9, 1)      # gripper open
  time.sleep(1)
  print()

  # short instructions
  print("\nChange joint angles using arrow keys to calibrate")
  print("Pay special attention to crosses formed by 4 screws")
  print("Hit ENTER to go to next step, q to quit\n")

  # shoulder calibration  
  hi = solicit(5,  45, "Shoulder - exactly horizontal: ")
  lo = solicit(5, -45, "           exactly vertical:   ")
  off, sc = swing(lo, hi)
  bot.SetCal(5, off, sc)
  print("    off %d, sc %5.2f\n" % (off, sc))

  # elbow calibration
  lo = solicit(4, -45, "Elbow - exactly vertical (align to shoulder): ")
  hi = solicit(4,  45, "        exactly horizontal (perpendicular):   ")
  off, sc = swing(lo, hi)
  bot.SetCal(4, off, sc)
  print("    off %d, sc %5.2f\n" % (off, sc))

  # wrist calibration 
  lo = solicit(3, -45, "Wrist - exactly vertical (perpendicular):    ")
  hi = solicit(3,  45, "        exactly horizontal (align to elbow): ")
  off, sc = swing(lo, hi)
  bot.SetCal(3, off, sc)
  print("    off %d, sc %5.2f\n" % (off, sc))

  # gripper calibration
  hi = solicit(1,  14.9, "Gripper: open jaws to 46 mm (1.8\") under pressure: ")
  lo = solicit(1, -14.9, "         close jaws until touching under pressure: ")
  off, sc = swing(lo, hi, 29.8)
  bot.SetCal(1, off, sc)
  print("    off %d, sc %5.2f\n" % (off, sc))

  # shoulder and elbow horizontal, wrist down
  bot.Pose(0, 45, -45, -45, -13.3, 1)
  time.sleep(1)

  # base calibration
  hi = solicit(6,   0, "Base - point arm straight forward:  ") 
  lo = solicit(6, -45, "       grasp to 85 mm (3.3\") right: ")
  off = hi - 1500
  sc = (hi - lo) / 45.0
  sc = round(sc, 2)
  bot.SetCal(6, off, sc)
  print("    off %d, sc %5.2f\n" % (off, sc))

  # record values more permanently
  bot.SaveCal(1)
  print("Saved new calibration values")

