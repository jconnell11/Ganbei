#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# spout_wrap.py : Python wrapper for speech output on MasterPi robot
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

import time                            # for testing
from ctypes import CDLL

import os
here = os.path.dirname(__file__)       
lib = CDLL(here + '/libmpi_spout.so')   


# Python wrapper for speech output on MasterPi robot

class MpiSpout:

  # create and configure Text-to-Speech system
  # if sh > 0 then shifts frequency more or less than 100%
  # returns 1 if successful, 0 or negative for problem

  def Start(self, sh =120):
    return lib.mpi_start(sh)


  # convert text to audio and start playing (does not block)

  def Say(self, txt):
    lib.mpi_say(txt.encode())


  # determine raw color code for sonar LEDs at the current time
  # returns pre-corrected 0xRRGGBB value, negative if not talking

  def Mouth(self):
    return lib.mpi_mouth()


  # cleanly shut down system

  def Done(self):
    lib.mpi_done()


# =========================================================================

# simple test program for single utterance

if __name__ == "__main__":
  tts = MpiSpout();
  old = -1
  col = -1
  tts.Start()
  print('--- hello there dude ---')
  tts.Say("hello there dude")
  while col < 0:                       # wait for audio start
    time.sleep(0.033)
    col = tts.Mouth()
  while col >= 0:                      # wait for audio finish
    if col != old:
      print('LED = ' + hex(col))       # show change of color
      old = col
    time.sleep(0.033)
    col = tts.Mouth()
  tts.Done()
  print('--- audio done ---')
   