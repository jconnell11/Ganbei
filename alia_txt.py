#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# alia_txt.py : Python wrapper for ALIA linguistic reasoning system
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
#
# Copyright 2023 Etaoin Systems
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

from ctypes import CDLL, c_char_p

lib = CDLL('./lib/libalia_txt.so')


# Python wrapper for ALIA linguistic reasoning system

class AliaTxt:

  # initialize state
  def __init__(self):
    self.inp = None
    self.out = None

  # convert string (incl. None) to char*
  def _s2c(self, s):
    return s if s == None else s.encode()

  # convert char* (incl. NULL) to string 
  def _c2s(self, c):
    return c if c == None else c.decode()

  # -----------------------------------------------------------------------

  # reset processing state at the start of a run
  # dir: base directory for config, language, log, and KB subdirectories
  # rname: robot name (like "Jim Jones"), can be NULL if desired
  # quiet: 1 = no console printing (only log), 0 = copious status messages
  # returns 1 if successful, 0 or negative for error
  def Reset(self, path, rname, silent):
    return lib.alia_reset(self._s2c(path), self._s2c(rname), silent)

  # do reasoning using any language input and recent sensor data
  # sets up language output and actuator commands to issue
  # if pace > 0 then delays return until next sensor cycle
  def Respond(self, pace):
    txt = self._s2c(self.inp)
    lib.alia_respond.restype = c_char_p
    msg = c_char_p(lib.alia_respond(txt))
    self.out = self._c2s(msg.value)
    self.inp = None
    lib.alia_daydream(pace);

  # stop processing and possibly save state at end of run
  # returns 1 if successful, 0 for error
  def Done(self, save):
    return lib.alia_done(save)

  # ----------------------------------------------------------------------- 

  # accept new typed text string
  def Txt_in(self, txt):
    self.inp = txt

  # accept new input string from speech
  # returns version with errors fixed up
  def Sp_in(self, txt):   
    self.inp = txt
    return txt
    
  # ----------------------------------------------------------------------- 

  # report new typed text message
  def Txt_out(self):
    return self.out

  # report new fixed up speech output message
  def Sp_out(self):
    return self.out

# =========================================================================

# simple test program
if __name__ == "__main__":
  a = AliaTxt();
  a.Reset(None, 'Joe Ganbei', 1)
  for i in range(50):
    if i == 20:
      a.Txt_in('What is your name?')
    a.Respond(1)
    msg = a.Txt_out()
    if not msg == None:
      print('  loop ' + str(i) + ' -> Robot says: ' + msg)
  a.Done(0)      
