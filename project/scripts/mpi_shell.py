#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# mpi_shell.py : interface to MasterPi robot miscellaneous components
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

import time                            # for testing

from threading import Thread, Event, Lock

from mpi_hiwonder import MasterPi, PlaySFX, LowBatt        


# -------------------------------------------------------------------------

# interface to MasterPi robot miscellaneous components
# background thread ramps lights up and down

class MpiShell(Thread):

  # initialize state (takes robot interface object as argument)
  def __init__(self, mpi):
    super(MpiShell, self).__init__()
    self.halt = Event()
    self.lock = Lock()
    self.bot = mpi           # for voltage

    # respiration and cycle rates
    self.bpm = 16.0
    self.hz  = 30.0

    # derived timing values
    self.wait = 1.0 / self.hz
    self.half = int(30.0 * self.hz / self.bpm + 0.5)
    self.full = 2 * self.half
    self.step = 1.0 / self.half

    # brightness variation
    self.mth1 = 0.5
    self.mth0 = 0.0

    # breathing color (orange)
    self.rmth = 1.0
    self.gmth = 0.5
    self.bmth = 0.0
    self.pend = -1           # no change queued

    # initial breathing state 
    self.cnt = 0
    self.brite = 1.0
    self.force = -1

    # battery monitoring
    self.vest = 0.0
    self.vchk = 60
    self.vhys = 0
    self.nag  = 0

    # set beeping threshold
    self.v10 = LowBatt()

    # start background thread
    self.start()


  # -----------------------------------------------------------------------

  # override Thread.run() which is called by start()

  def run(self):
    while not self.halt.is_set():
      self.ramp()
      self.lights()
      self.voltage()
      time.sleep(self.wait)
    self.Talk(0)


  # vary "brite" from 0 to 1 linearly

  def ramp(self):
    inc = -self.step if self.cnt < self.half else self.step
    self.brite += inc
    self.brite = max(0.0, min(self.brite, 1.0))
    self.cnt += 1
    if self.cnt >= self.full:
      self.cnt = 0
      self.brite = 1.0


  # change intensity of LEDs 
  # switch color at darkest part of cycle

  def lights(self):
    with self.lock:
      if self.pend > 0:
        if self.cnt == self.half or self.force >= 0:
          self.rmth = self.rmth2
          self.gmth = self.gmth2
          self.bmth = self.bmth2
          self.pend = 0
      if self.force >= 0:              # talk or flash ongoing
        self.cnt = self.half
        self.brite = 0.0
        return
    v = self.brite * (self.mth1 - self.mth0) + self.mth0
    col = self.gamma(v * self.rmth, v * self.gmth, v * self.bmth)
    self.bot.Eyes(col)
    self.bot.Body(col)


  # get an R:G:B color value with approximate gamma correction
  # acts to squash low values (e.g. rf 0.50 -> red 81 not 128)

  def gamma(self, rf, gf, bf):
    e = 1.65
    red = int(255.0 * pow(rf, e) + 0.5)
    red = max(0, min(red, 255))
    grn = int(255.0 * pow(gf, e) + 0.5)
    grn = max(0, min(grn, 255))
    blu = int(255.0 * pow(bf, e) + 0.5)
    blu = max(0, min(blu, 255))
    return (red << 16) | (grn << 8) | blu


  # get a smoothed estimate of battery voltage
  # essentially duplicated from mpi_buttons.py

  def voltage(self):

    # if blip started turn off buzzer after several cycles
    if self.nag > 0:
      self.nag -= 1
      if self.nag == 0:
        self.bot.Beep(0, 1)            # for 2023 expansion board

    # read voltage once every 2 seconds (60 cycles at 30 Hz)
    self.vchk += 1
    if self.vchk < 60:                
      return

    # make sure sample not crazy (often!)
    self.vchk = 0
    v = self.bot.Voltage() 
    with self.lock:
  
      # if sample not crazy (often!) add to IIR filter 
      if v >= 5.0 and v <= 8.5:
        if self.vest <= 0.0:
          self.vest = v                # quick start
        else:
          self.vest += 0.2 * (v - self.vest)
      elif self.vest <= 0:
        return

      # if less than 10% left start beep                           
      if self.vest > self.v10 + 0.2:                   
        self.vhys = 0
      elif self.vhys > 0 or self.vest < self.v10:  
        self.vhys = 1   
        PlaySFX("beep2", 0, 1)
        self.bot.Beep(1, 1)            # backup beep (200 ms)
        self.nag = 4                    


  # -----------------------------------------------------------------------

  # define basic color for pulsing (e.g. for mood)

  def Breath(self, rgb):   
    r = (rgb & 0xFF0000) >> 16  
    g = (rgb & 0x00FF00) >> 8
    b =  rgb & 0x0000FF
    r *= 1.3                           # red LED is weaker than others
    top = max(r, max(g, b))
    if top > 0.0:
      b /= top
      g /= top
      r /= top
    with self.lock:
      if self.pend >= 0:
        if self.rmth2 == r and self.gmth2 == g and self.bmth2 == b:
          return
      self.rmth2 = r
      self.gmth2 = g 
      self.bmth2 = b 
      self.pend = 1


  # immediately set LEDs to white (e.g. for surprise)

  def Flash(self, doit):
    col = 0xFAFF80           # special - not used externally
    if doit > 0:
      self.Talk(col)
    else:
      with self.lock:
        if self.force == col:
          self.force = -1


  # directly set LEDs to some pre-corrected R:G:B value

  def Talk(self, col):  
    with self.lock:
      if col < 0:
        self.force = -1
        return
      if col == self.force:
        return
      self.force = col  
      self.bot.Eyes(col)
      self.bot.Body(col) 


  # report current battery voltage
  # Li-ion under load: 7.8v = 100%, 6.6v = 10%

  def Battery(self):
    with self.lock:
      if self.vest <= 0.0:
        return 8.0           # assume fully charged
      return self.vest


  # signal loop to cleanly terminate then wait for it

  def Done(self):
    self.halt.set()
    Thread.join(self, None)


# =========================================================================

# simple test runs breathing cycle

if __name__ == "__main__":
  bot = MasterPi()
  b = MpiShell(bot)

  # speech sequence
#  b.Talk(0)
#  time.sleep(0.25)
#  b.Talk(0x502020)          # talk
#  time.sleep(0.2)
#  b.Talk(0)
#  time.sleep(0.2)
#  b.Talk(0x502020)          # talk
#  time.sleep(0.4)
#  b.Talk(0)
#  time.sleep(0.25)
#  b.Talk(0x004020)          # listen
#  time.sleep(3)
#  b.Talk(-1)
#  time.sleep(10.0)
  
  # lonely (blue)
#  b.Breath(0x0040FF, 0.5)   
#  time.sleep(10.0)

  # happy (greenish)
#  b.Breath(0x60FF00, 0.5) 
#  time.sleep(10.0)

  # bored (purple)
#  b.Breath(0x8000FF, 0.5)     
#  time.sleep(10.0)

  # scared (yellow)
#  b.Breath(0xE0FF00, 1.0)     
#  time.sleep(10.0)

  # angry (red)
#  b.Breath(0xFF2000, 1.0)     
#  time.sleep(10.0)

  # surprise (white)
#  b.Breath(0xC0FFC0, 1.0)     
#  time.sleep(10.0)

 
  # test breathing color and direct modulation
  print("start green breathing")
  b.Breath(0x00FF00)  
  time.sleep(7)

  print("  cyan")
  b.Talk(0x008080)        
  time.sleep(2)
  b.Talk(-1)
  print("resume breathing")
  time.sleep(3)

  print("  red");
  b.Talk(0x800000)          
  time.sleep(2)
  b.Talk(-1)
  print("resume breathing")
  time.sleep(3)

  print("  orange")
  b.Talk(0xFF8000)          
  time.sleep(2)
  b.Talk(-1)
  print("resume breathing")
  time.sleep(3)

  print("switch to magenta breathing")
  b.Breath(0xFF00FF)         
  time.sleep(5)

  print("  white flash")
  b.Flash(1);                
  time.sleep(1)
  b.Flash(0);
  print("resume breathing")
  time.sleep(5)

  print("off")
  b.Talk(0)
  time.sleep(3)
  print("done")
  b.Done()

