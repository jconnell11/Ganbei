#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# mpi_buttons.py : monitors expansion board buttons and checks battery
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

import os, sys, time
import RPi.GPIO as GPIO

sys.path.append('/home/pi/MasterPi/HiwonderSDK')
import Board


# ----------------------------------------------------------------------- 

# clear voltage estimate and beeping state
voltage = 8.0
volt_cnt = 40
volt_nag = 0
volt_hys = 0


# check average battery voltage and beep if low
# no easy way to share this with main application

def check_battery():
  global voltage, volt_cnt, volt_nag, volt_hys

  # if blip started turn off buzzer after several cycles
  if volt_nag > 0:
    volt_nag -= 1
    if volt_nag == 0:
      Board.setBuzzer(0)

  # read voltage once every 2 seconds (40 cycles at 20 Hz)
  volt_cnt += 1
  if volt_cnt < 40:
    return
  volt_cnt = 0 
    
  # if sample not crazy (often!) add to IIR filter 
  v = Board.getBattery() / 1000.0
  if v > 5.0 and v < 8.5:
    voltage += 0.2 * (v - voltage)

  # if too low then start beep 
  if voltage > 7.2:                    # nearly latched
    volt_hys = 0
  elif volt_hys > 0 or voltage < 6.7:  # was 6.8V for Hiwonder
    volt_hys = 1          
    Board.setBuzzer(1)
    volt_nag = 4                       # medium beep (200ms)


# ----------------------------------------------------------------------- 

# configure button inputs (default to BCM mode pin numbers)

GPIO.setwarnings(False)
key1_pin = 13
key2_pin = 23
mode = GPIO.getmode()
if mode == None or mode < 0:
  GPIO.setmode(GPIO.BCM)              # preferred
elif mode == GPIO.BOARD:
  key1_pin = 33                 
  key2_pin = 16
GPIO.setup(key1_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(key2_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)


# clear key timing state
key1_cnt = 0
key2_cnt = 0


# check state of buttons on expansion board
# front brief = start demo, front 3 sec = <nothing>
#  back brief = stop demo,   back 3 sec = shutdown

def check_keys():
  global key1_pin, key2_pin, key1_cnt, key2_cnt

  # check if key1 currently being pressed
  if GPIO.input(key1_pin) == GPIO.LOW:
    key1_cnt += 1
    key2_cnt = 0
    if key1_cnt == 60:
      print('switch wifi mode')
      Board.setBuzzer(1)
      time.sleep(0.4)                  # long beep
      Board.setBuzzer(0)
    return    
  
  # key1 not currently pressed
  if key1_cnt > 0:
    print('start new demo')
    Board.setBuzzer(1)
    time.sleep(0.1)                    # short beep
    Board.setBuzzer(0)
    time.sleep(0.1)
    Board.setBuzzer(1)
    time.sleep(0.1)                    # short beep
    Board.setBuzzer(0)
    stop_demo()
    start_demo()
  key1_cnt = 0 

  # check if key2 currently being pressed
  if GPIO.input(key2_pin) == GPIO.LOW:
    key2_cnt += 1
    if key2_cnt == 60:
      print('system shutdown')
      Board.setBuzzer(1)
      time.sleep(0.4)                  # long beep
      Board.setBuzzer(0)
      stop_demo()
      os.system('shutdown -h now')
    return    
  
  # key2 not currently pressed
  if key2_cnt > 0:
    print('stop any demo')
    Board.setBuzzer(1)
    time.sleep(0.1)                    # short beep
    Board.setBuzzer(0)
    stop_demo()
  key2_cnt = 0 


# launch current demo (whatever it is)
# connect to program: sudo screen -dr

def start_demo():      
  os.system("cd /home/pi/Ganbei;screen -dm python3 Ganbei_act.py")


# make sure demo program has stopped cleanly

def stop_demo():
  os.system("pkill -2 -f Ganbei_act.py")


# =========================================================================

# check buttons and voltage at 20 Hz

if __name__ == "__main__":
  while True:
    check_battery()
    check_keys()
    time.sleep(0.05)
