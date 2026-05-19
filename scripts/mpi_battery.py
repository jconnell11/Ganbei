#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# mpi_buttons.py : checks battery and beeps if too low
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

from mpi_hiwonder import MasterPi, PlaySFX, LowBatt


# checks battery and beeps if too low

class MpiBattery:

  # initialize state (takes robot interface object as argument)
  def __init__(self, mpi):
    self.bot = mpi
    self.voltage = 0.0
    self.volt_cnt = 40
    self.volt_hys = 0
    self.volt_nag = 0
    self.v10 = LowBatt(0)              # light processor load


  # check average battery voltage and beep if low

  def check_voltage(self):
   
    # if blip started turn off buzzer after several cycles
    if self.volt_nag > 0:
      self.volt_nag -= 1
      if self.volt_nag == 0:
        self.bot.Beep(0)               # for 2023 expansion board

    # read voltage once every 2 seconds (40 cycles at 20 Hz)
    self.volt_cnt += 1
    if self.volt_cnt < 40:
      return
    self.volt_cnt = 0 

    # if sample not crazy (often!) add to IIR filter 
    v = bot.Voltage()
    print("v = %4.2f [%4.2f]" % (v, self.voltage), flush=True)
    if v >= 5.0 and v <= 8.5:
      if self.voltage <= 0.0:
        self.voltage = v               # quick start
      else:
        self.voltage += 0.2 * (v - self.voltage)
    elif self.voltage <= 0:
      return

    # if less than 10% left start beep                            
    if self.voltage > self.v10 + 0.2:                   
      self.volt_hys = 0
    elif self.volt_hys > 0 or self.voltage < self.v10:  
      self.volt_hys = 1
      PlaySFX("beep2", 0)   
      self.bot.Beep(1)                 # backup beep (200 ms)
      self.volt_nag = 4                


# =========================================================================

# repeatedly voltage at 20 Hz (unless Ganbei using expansion board)
# can see voltage using: journalctl -f -u mpi_buttons.service

if __name__ == "__main__":
  try:
    bot = MasterPi(0)                  # fails if Ganbei running
    m = MpiBattery(bot)
    while True:
      m.check_voltage()
      time.sleep(0.05)
  except:
    print("Bad connection to expansion board", flush=True)
   