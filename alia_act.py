#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# alia_act.py : Python wrapper for ALIA linguistic reasoning system
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
#
# Copyright 2024 Etaoin Systems
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

import socket, time                                     
from ctypes import CDLL, c_char_p, c_int, c_float

lib = CDLL('./lib/libalia_act.so')
lib.alia_spout.restype = c_char_p


# Python wrapper for ALIA linguistic reasoning system
# basically has a big pile of gettable and settable class variables
# NOTE: either do "sudo chmod a+w KB log dump" or run with "sudo" for files

class AliaAct:

  # allow access to C++ command and sensor variables
  # read commands with ".value" and write sensores with ".value ="
  def __init__(self):

    # ----------------------------- SPEECH ----------------------------------

    self.Attn = c_int.in_dll(lib, "alia_attn")     # paying attention (no wake)

    self.Hear = c_int.in_dll(lib, "alia_hear")     # currently hearing speech 
    self.Talk = c_int.in_dll(lib, "alia_talk")     # busy with TTS talking now

    # ------------------------------ BODY -----------------------------------
 
    self.Mood = c_int.in_dll(lib, "alia_mood")     # mood bit vector

    self.Batt = c_float.in_dll(lib, "alia_batt")   # battery capacity percent
    self.Tilt = c_float.in_dll(lib, "alia_tilt")   # fwd/back vehicle tilt now
    self.Roll = c_float.in_dll(lib, "alia_roll")   # CCW/CW vehicle roll now

    # ------------------------------ NECK -----------------------------------

    self.Npt = c_float.in_dll(lib, "alia_npt")     # desired pan angle
    self.Ntt = c_float.in_dll(lib, "alia_ntt")     # desired tilt angle 
    self.Npv = c_float.in_dll(lib, "alia_npv")     # pan rate wrt normal
    self.Ntv = c_float.in_dll(lib, "alia_ntv")     # tilt rate wrt normal
    self.Npi = c_int.in_dll(lib,   "alia_npi")     # pan cmd importance
    self.Nti = c_int.in_dll(lib,   "alia_nti")     # tilt cmd importance

    self.Nx = c_float.in_dll(lib, "alia_nx")       # current camera X
    self.Ny = c_float.in_dll(lib, "alia_ny")       # current camera Y
    self.Nz = c_float.in_dll(lib, "alia_nz")       # current camera Z
    self.Np = c_float.in_dll(lib, "alia_np")       # camera pan angle now
    self.Nt = c_float.in_dll(lib, "alia_nt")       # camera tilt angle now
    self.Nr = c_float.in_dll(lib, "alia_nr")       # camera roll angle now

    # ------------------------------ LIFT ----------------------------------

    self.Fht = c_float.in_dll(lib, "alia_fht")     # desired fork height
    self.Fhv = c_float.in_dll(lib, "alia_fhv")     # height rate wrt normal
    self.Fhi = c_int.in_dll(lib,   "alia_fhi")     # lift cmd importance

    self.Fh = c_float.in_dll(lib, "alia_fh")       # fork height now

    # ------------------------------- ARM -----------------------------------

    self.Axt = c_float.in_dll(lib, "alia_axt")     # desired gripper X
    self.Ayt = c_float.in_dll(lib, "alia_ayt")     # desired gripper Y
    self.Azt = c_float.in_dll(lib, "alia_azt")     # desired gripper Z
    self.Apt = c_float.in_dll(lib, "alia_apt")     # desired gripper pan
    self.Att = c_float.in_dll(lib, "alia_att")     # desired gripper tilt
    self.Art = c_float.in_dll(lib, "alia_art")     # desired gripper roll
    self.Awt = c_float.in_dll(lib, "alia_awt")     # desired width (force)
    self.Apv = c_float.in_dll(lib, "alia_apv")     # position shift rate 
    self.Adv = c_float.in_dll(lib, "alia_adv")     # direction slew rate
    self.Awv = c_float.in_dll(lib, "alia_awv")     # width change rate
    self.Ajv = c_float.in_dll(lib, "alia_ajv")     # tuck joints angular rate
    self.Apm = c_int.in_dll(lib,   "alia_apm")     # position mode bits
    self.Adm = c_int.in_dll(lib,   "alia_adm")     # direction mode bits
    self.Api = c_int.in_dll(lib,   "alia_api")     # position cmd importance
    self.Adi = c_int.in_dll(lib,   "alia_adi")     # direction cmd importance
    self.Awi = c_int.in_dll(lib,   "alia_awi")     # width cmd importance
    self.Aji = c_int.in_dll(lib,   "alia_aji")     # tuck joints cmd importance

    self.Ax = c_float.in_dll(lib, "alia_ax")       # gripper X now
    self.Ay = c_float.in_dll(lib, "alia_ay")       # gripper Y now
    self.Az = c_float.in_dll(lib, "alia_az")       # gripper Z now
    self.Ap = c_float.in_dll(lib, "alia_ap")       # gripper pan now
    self.At = c_float.in_dll(lib, "alia_at")       # gripper tilt now
    self.Ar = c_float.in_dll(lib, "alia_ar")       # gripper roll now
    self.Aw = c_float.in_dll(lib, "alia_aw")       # gripper width now
    self.Af = c_float.in_dll(lib, "alia_af")       # gripper force now
    self.Aj = c_float.in_dll(lib, "alia_aj")       # tuck joint error now

    # ------------------------------ BASE -----------------------------------

    self.Bmt = c_float.in_dll(lib, "alia_bmt")     # incremental move amount
    self.Brt = c_float.in_dll(lib, "alia_brt")     # incremental turn amount
    self.Bsk = c_float.in_dll(lib, "alia_bsk")     # move direction wrt fwd
    self.Bmv = c_float.in_dll(lib, "alia_bmv")     # desired move rate
    self.Brv = c_float.in_dll(lib, "alia_brv")     # desired rotation rate

    self.Bx = c_float.in_dll(lib, "alia_bx")       # map X location
    self.By = c_float.in_dll(lib, "alia_by")       # map Y location
    self.Bh = c_float.in_dll(lib, "alia_bh")       # map CCW heading

  # -------------------------------------------------------------------------

  # specify which hardware susbsystems are present and working

  def Body(self, neck, arm, lift, base):
    lib.alia_body(neck, arm, lift, base)


  # configure reasoning system and load knowledge base
  # app_name: name of program to print on console at beginning
  # makes file "config/all_names.txt" for speech recognition
  # returns 1 if okay, 0 or negative for problem

  def Reset(self, app_name):
    dir = '/home/pi/Ganbei'
    rname = socket.gethostname() + ' Ganbei'
    return lib.alia_reset(dir.encode(), rname.encode(), app_name.encode())


  # exchange command and sensor data then start reasoning a bit  
  # returns 2 if okay, 1 if not ready, 0 for quit, negative for problem

  def Think(self):
    return lib.alia_think()


  # cleanly stop reasoning system and possibly save knowledge base
  # returns 1 if okay, 0 or negative for problem

  def Done(self, save):
    return lib.alia_done(save)


  # -------------------------------------------------------------------------

  # text output from reasoner for TTS

  def Spout(self):
    return lib.alia_spout().decode()


  # text input to reasoner from speech recognition

  def Spin(self, reco):
    lib.alia_spin(c_char_p(reco.encode()))


# =========================================================================

# simple test program

if __name__ == "__main__":
  ai = AliaAct()
  ai.Reset('alia_act')
  cnt = 0
  last = '<none>'
  while ai.Think() > 0:
    msg = ai.Spout()
    if not msg == '':
      last = msg
    cnt += 1
    if cnt == 60:                        # 2 sec
      ai.Spin('Herbie, what are you?')
    time.sleep(0.033)
  ai.Done(1)   
  print('last msg = ' + last)   
