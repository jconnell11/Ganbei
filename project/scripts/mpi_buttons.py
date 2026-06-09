#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# mpi_buttons.py : monitors expansion board buttons (only)
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

import os, time, gpiod       

from mpi_hiwonder import PlaySFX


# monitors expansion board buttons (only)

class MpiButtons:

  # initialize state
  def __init__(self):
    self.cfg_input()
    self.front_cnt = 0
    self.back_cnt = 0


  # clean up on exit (release GPIO)
  def __del__(self):
    self.f_but.release()
    self.b_but.release()
    self.chip.close()


  # configure button inputs

  def cfg_input(self):

    # link to proper GPIO chip
    try:
      self.chip = gpiod.Chip('gpiochip4')        # Raspberry Pi 5
    except:
      self.chip = gpiod.Chip('gpiochip0')        # Raspberry Pi 4B

    # get pin interfaces
    front_pin = 13                               # key1 
    back_pin  = 23                               # key2 (broken?)
    self.f_but = self.chip.get_line(front_pin)
    self.b_but = self.chip.get_line(back_pin)

    # both need pullups for switches 
    input  = gpiod.LINE_REQ_DIR_IN
    pullup = gpiod.LINE_REQ_FLAG_BIAS_PULL_UP
    self.f_but.request(consumer="key1", type=input, flags=pullup)
    self.b_but.request(consumer="key2", type=input, flags=pullup)


  # ----------------------------------------------------------------------- 

  # check state of buttons on expansion board
  #   front: brief = start demo, 3 sec = shutdown
  #    back: brief = stop demo,  3 sec = reboot

  def check_keys(self):

    # check if front button currently being pressed
    if self.f_but.get_value() == 0:
      time.sleep(0.05)                   # noise reject
      if self.f_but.get_value() == 0:
        self.front_cnt += 1
        self.back_cnt = 0
        if self.front_cnt == 30:         # 30 * 2 * 0.05 = 3 sec
          print('system shutdown')
          PlaySFX("beep4")               
          self.stop_demo()                      
          os.system('sudo shutdown -h now')    # ==> SHUTDOWN 
          self.front_cnt = -20                 # repeat at 5 sec
        return    
  
    # front button no longer pressed
    if self.front_cnt > 0:
      print('start new demo')
      PlaySFX("beep_beep")                         
      self.stop_demo()                        
      self.start_demo()                  # ==> START DEMO
    self.front_cnt = 0 

    # check if back button currently being pressed
    if self.b_but.get_value() == 0:
      time.sleep(0.05)                   # noise reject
      if self.b_but.get_value() == 0:
        self.back_cnt += 1
        if self.back_cnt == 30:          # 30 * 2 * 0.05 = 3 sec
          print('system reboot')
          PlaySFX("beep4_beep")         
          self.stop_demo()                      
          os.system('sudo reboot')       # ==> REBOOT 
          self.back_cnt = -20            # repeat at 5 sec
        return    
  
    # back button no longer pressed
    if self.back_cnt > 0:
      print('stop any demo')
      PlaySFX("beep")                    
      self.stop_demo()                   # ==> STOP DEMO
    self.back_cnt = 0 


  # ----------------------------------------------------------------------- 

  # launch current demo (whatever it is)
  # connect to program: screen -dr

  def start_demo(self):      
    os.system("screen -dm bash -c demo")


  # make sure demo program has stopped cleanly

  def stop_demo(self):
    os.system("pkill -INT -f Ganbei_vis.py")


# =========================================================================

# repeatedly check buttons at 20 Hz (even when Ganbei is running)

if __name__ == "__main__":
  m = MpiButtons()
  while True:
    m.check_keys()
    time.sleep(0.05)
   