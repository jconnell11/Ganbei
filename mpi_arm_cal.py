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

import time, yaml, socket, sys
import getch             # sudo pip3 install getch

sys.path.append('/home/pi/MasterPi/HiwonderSDK')
import Board
 

# -------------------------------------------------------------------------

# load "deviation" file into global servo offset variables

def load_devs():  
  global goff, woff, eoff, soff, boff
  with open("/home/pi/MasterPi/Deviation.yaml", 'r') as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
  goff = data.get('1', 0)
  woff = data.get('3', 0)
  eoff = data.get('4', 0)
  soff = data.get('5', 0)
  boff = data.get('6', 0)

# save global servo offset variables to "deviation" file

def save_devs():
  global goff, woff, eoff, soff, boff
  with open("/home/pi/MasterPi/Deviation.yaml", 'w') as f:
    yaml.dump({'1':goff, '3':woff, '4':eoff, '5':soff, '6':boff}, f)
  print("Saved servo offsets in: MasterPi/Deviation.yaml")


# load "calib" file into global servo angular scale factors
# default scaling = 2000us / 180 degs = 11.1

def load_scale():
  global gsc, wsc, esc, ssc, bsc
  cfile = "config/" + socket.gethostname() + "_servo.yaml"
  with open(cfile, 'r') as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
  gsc = data.get('g_sc', 13.6)
  wsc = data.get('w_sc', 11.1)
  esc = data.get('e_sc', 11.1)
  ssc = data.get('s_sc', 11.1)
  bsc = data.get('b_sc', 11.1)


# save global servo angular scale variables to "calib" file
# file has other values that must be preserved

def save_scale():
  global gsc, wsc, esc, ssc, bsc
  cfile = "config/" + socket.gethostname() + "_servo.yaml"
  with open(cfile, 'r') as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
  data['g_sc'] = gsc
  data['w_sc'] = wsc
  data['e_sc'] = esc
  data['s_sc'] = ssc
  data['b_sc'] = bsc
  with open(cfile, 'w') as f:
    yaml.dump(data, f, sort_keys=False)
  print("Saved scale factors in: %s" % (cfile))


# -------------------------------------------------------------------------

# change some joint pulse width using arrow keys 
# stops whole program if 'q', exits if 'enter'

def solicit(sid, pw0, prompt):

  # set servo to initial angle
  pw = int(pw0 + 1500)
  Board.setPWMServoPulse(sid, pw, 1000)
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
    Board.setPWMServoPulse(sid, pw - 50, 0)
    time.sleep(0.2)
    Board.setPWMServoPulse(sid, pw, 0)

  # end prompt line and give best value
  print()
  return pw


# determine zero offset width and scaling for servo based on lo and hi
# "dev" is the current calibration from the "deviation" file
# assumes "degs" is symmetric around servo neutral (1500us) position

def swing(dev, lo, hi, degs =90):
  sc  = (hi - lo) / degs
  sc = round(sc, 2)
  off = int((hi + lo) / 2 - 1500)
  off += dev
  print("    dev %d, sc %5.2f\n" % (off, sc))
  return off, sc


# =========================================================================

# adjust arm with keyboard and save calibration values

if __name__ == "__main__":

  # read default values
  global goff, woff, eoff, soff, boff
  global gsc, wsc, esc, ssc, bsc
  load_devs()
  load_scale()

  # initialize arm pose
  print("Posing arm ... ", end="")
  sys.stdout.flush()
  Board.setPWMServoPulse(4, 1000, 1000)          # elbow inline
  time.sleep(1)
  Board.setPWMServoPulse(5, 1500, 1000)          # shoulder diagonal up
  time.sleep(1)
  Board.setPWMServoPulse(3, 1000, 1000)          # wrist inline
  time.sleep(1)
  Board.setPWMServoPulse(6, 1500, 1000)          # arm forward
  time.sleep(1)
  Board.setPWMServoPulse(1, 1648, 1000)          # gripper open
  time.sleep(1)
  print()

  # short instructions
  print("Change joint angles using arrow keys to calibrate")
  print("Pay special attention to crosses formed by 4 screws")
  print("Hit ENTER to go to next step, q to quit\n")

  # shoulder calibration  
  hi = solicit(5,  45 * ssc, "Shoulder - exactly horizontal: ")
  lo = solicit(5, -45 * ssc, "           exactly vertical:   ")
  soff, ssc = swing(soff, lo, hi)

  # elbow calibration
  hi = solicit(4,  45 * esc, "Elbow - bend perpendicular (horizontal):  ")
  lo = solicit(4, -45 * esc, "        align with lower link (vertical): ")
  eoff, esc = swing(eoff, lo, hi)

  # wrist calibration
  hi = solicit(3,  45 * wsc, "Wrist - align jaws with link (vertical): ")
  lo = solicit(3, -45 * wsc, "        bend perpendicular (horizontal): ")
  woff, wsc = swing(woff, lo, hi)

  # gripper calibration
  hi = solicit(1,  13.3 * gsc, "Gripper: open jaws to 46mm (1.8\") under pressure:  ")
  lo = solicit(1, -13.3 * gsc, "         close jaws until touching under pressure: ")
  goff, gsc = swing(goff, lo, hi, 26.6)

  # base calibration (shoulder horizontal)
  Board.setPWMServoPulse(5, int(1500 + soff + 45 * ssc), 1000)
  time.sleep(1)
  hi = solicit(6,   0,       "Base - point arm straight forward: ") 
  boff += hi - 1500
  lo = solicit(6, -45 * bsc, "       grasp to 85mm (3.3\") right: ")
  bsc = (hi - lo) / 45.0
  bsc = round(bsc, 2)
  print("    dev %d, sc %5.2f\n" % (boff, bsc))

  # record values
  save_devs()
  save_scale()




