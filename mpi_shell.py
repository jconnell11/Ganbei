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

from threading import Thread, Event, Lock
import time, sys

sys.path.append('/home/pi/MasterPi/HiwonderSDK')
import Board
import Sonar


# interface to MasterPi robot miscellaneous components
# background thread ramps body lights up and down
# Board needs "sudo" to allow rpi_ws281x.so to access /dev/mem

class MpiShell(Thread):

  # create components
  def __init__(self):
    super(MpiShell, self).__init__()
    self.halt = Event()
    self.lock = Lock()
    self.sn = Sonar.Sonar()

    # respiration and cycle rates
    self.bpm = 16.0
    self.hz  = 30.0

    # derived timing values
    self.wait = 1.0 / self.hz
    self.half = int(30.0 * self.hz / self.bpm + 0.5)
    self.full = 2 * self.half
    self.step = 1.0 / self.half

    # brightness variation
    self.bod1 = 1.0
    self.bod0 = 0.2

    # eyes (off) and pulsing color (orange)
    self.Talk(0)
    self.rbod = 1.0
    self.gbod = 0.39
    self.bbod = 0.0
    self.pend = -1           # nothing queued

    # initial breathing state
    self.cnt = 0
    self.brite = 1.0
    self.white = 0

    # battery monitoring
    self.vest = 8.0
    self.vchk = 60

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
    Board.RGB.setPixelColor(0, 0)   
    Board.RGB.setPixelColor(1, 0)
    Board.RGB.show()


  # vary "brite" from 0 to 1 linearly

  def ramp(self):
    inc = -self.step if self.cnt < self.half else self.step
    self.brite += inc
    self.brite = max(0.0, min(self.brite, 1.0))
    self.cnt += 1
    if self.cnt >= self.full:
      self.cnt = 0
      self.brite = 1.0


  # change intensity of body and sonar LEDs 
  # switch color at darkest part of cycle
  # needs sudo for ws281x to access /dev/mem

  def lights(self):
    with self.lock:
      if self.pend > 0:
        if self.cnt == self.half or self.white != 0:
          self.rbod = self.rbod2
          self.gbod = self.gbod2
          self.bbod = self.bbod2
          self.pend = 0
      if self.white != 0:              # flash ongoing
        self.cnt = self.half
        self.brite = 0.0
        return
    v = self.brite * (self.bod1 - self.bod0) + self.bod0
    col = self.gamma(v * self.rbod, v * self.gbod, v * self.bbod)
    Board.RGB.setPixelColor(0, col)
    Board.RGB.setPixelColor(1, col)
    Board.RGB.show()


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
  # essentially duplicated from mpi_buttons

  def voltage(self):
    self.vchk += 1
    if self.vchk < 60:                 # 2 sec
      return
    self.vchk = 0
    v = Board.getBattery() / 1000.0
    if v > 5.0 and v < 8.5:            # skip crazy
      with self.lock:
        self.vest += 0.2 * (v - self.vest)


  # -----------------------------------------------------------------------

  # define basic color for pulsing (e.g. for mood)

  def Breath(self, rgb):   
    r = (rgb & 0xFF0000) >> 16  
    g = (rgb & 0x00FF00) >> 8
    b =  rgb & 0x0000FF
    r *= 1.3                 # red LED is weaker than others
    top = max(r, max(g, b))
    if top > 0.0:
      b /= top
      g /= top
      r /= top
    with self.lock:
      if self.pend >= 0:
        if self.rbod2 == r and self.gbod2 == g and self.bbod2 == b:
          return
      self.rbod2 = r
      self.gbod2 = g 
      self.bbod2 = b 
      self.pend = 1

  # immediately set body LEDs to white

  def Flash(self, doit):
    with self.lock:
      if doit == self.white:
        return
      self.white = doit
      c = 0 if doit == 0 else 0xFAFF80
      Board.RGB.setPixelColor(0, c)
      Board.RGB.setPixelColor(1, c)
      Board.RGB.show()


  # directly set sonar LEDs to some pre-correctd R:G:B value

  def Talk(self, col):    
    c = col if col > 0 else 0         
    self.sn.setPixelColor(0, c)
    self.sn.setPixelColor(1, c)
    self.sn.show()


  # report current battery voltage
  # Li-ion: 7.2v = 20%, 6.8v = 10%

  def Battery(self):
    with self.lock:
      v = self.vest
    return v


  # signal user with several short beeps

  def Beep(self, n):
    for i in range(n):
      if i > 0:
        time.sleep(0.1)
      Board.setBuzzer(1)
      time.sleep(0.1)
      Board.setBuzzer(0)


  # signal loop to cleanly terminate then wait for it

  def Done(self):
    self.halt.set()
    Thread.join(self, None)


# =========================================================================

# simple test runs breathing cycle

if __name__ == "__main__":
  b = MpiShell();
  time.sleep(5.0)

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

  
  # test breathing color and eye modulation
  b.Breath(0x00FF00)         # green breathing
  time.sleep(5.0)
  b.Talk(0x000080)           # blue eyes
  time.sleep(0.5)
  b.Talk(-1)
  time.sleep(0.5)
  b.Talk(0x800000)           # red eyes
  time.sleep(0.5)
  b.Talk(-1)
  time.sleep(0.5)
  b.Talk(0x808080)           # white eyes
  time.sleep(0.5)
  b.Talk(-1)
  b.Breath(0xFF00FF)         # magenta breathing
  time.sleep(5.0)
  b.Flash(1);                # white back
  time.sleep(1.0)
  b.Flash(0);
  time.sleep(5.0)
  b.Done()
