#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# Ganbei_act.py : control MasterPi robot with ALIA reasoner
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

import time, os, signal

from alia_act import AliaAct
from azure_reco import AzureReco
#from pico_reco.pico_reco import PicoReco
from mpi_spout.mpi_spout import MpiSpout

from mpi_shell import MpiShell
from mpi_arm import MpiArm
from mpi_imu import MpiImu
from mpi_base import MpiBase


# control MasterPi robot with ALIA reasoner
# must use "sudo python3 Ganbei_act.py" to access LEDs via /dev/mem
# need to modify Raspbian to run pulseaudio for all users (incl. root)

class GanbeiAct:

  # set up components and main loop control
  def __init__(self):

    # allow for graceful exit
    signal.signal(signal.SIGINT, self.Quit)
    signal.signal(signal.SIGTERM, self.Quit)

    # main loop timing (30Hz)
    self.tick = 0.0
    self.cycle = 1.0 / 30.0

    # turn on body LEDs very early
    self.body = MpiShell()

    # create reasoner and language components
    self.ai = AliaAct()
    self.reco = AzureReco()
#    self.reco = PicoReco()
    self.tts = MpiSpout()

    # interface to hardware components
    self.arm = MpiArm()
    self.base = MpiBase()
    self.imu = MpiImu()
    self.ai.Body(1, 1, 0, 1)

    # mouth LED state variables
    self.mth0 = -1
    self.col0 = -1

    # action speed factor
    self.sf = 1.0


  # request termination after next loop finishes
  # Note: can be called externally

  def Quit(self, *args):
    self.loop = False


  # ----------------------------- MAIN LOOP ------------------------------- 

  # update sensors, reason a bit, then issue commands
  # Note: meant to be called externally

  def Run(self):
    self.start()
    while self.loop:
      self.issue()         
      self.update()         
      if self.ai.Think() <= 0:
        break
      self.pace()     
    self.shutdown()


  # sleep until next cycle start time

  def pace(self):
    now = time.time()
    if self.tick == 0.0:
      self.tick = now
    wait = self.tick - now
    self.tick += self.cycle
    if (wait < 0):
      wait = 0
    time.sleep(wait)         # always allow thread swap


  # configure and start up all components

  def start(self):
    
    # clear audio then default failed flag
    print('Initializing ...')
    os.system("systemctl restart pulseaudio.service")
    os.system("pactl set-source-mute @DEFAULT_SOURCE@ 0 &")   
    self.loop = False

    # start speech I/O and reasoner
    if self.tts.Start() <= 0:
      print('No text-to-speech!')
      return
    if self.ai.Reset('Ganbei_act') <= 0:   # makes name list
      print('Problem with ALIA!')
      return
    if self.reco.Start() <= 0:             # needs name list
      print('No speech recognition!')
      self.body.Beep(3)

    # everything okay to run
    self.loop = True


  # transfer commands from ALIA reasoner to actuators

  def issue(self):
    self.tts_issue()
    self.body_issue()
    self.arm_issue()
    self.base_issue()


  # get data from sensors and transfer to ALIA reasoner

  def update(self):
    self.reco_update()
    self.body_update()
    self.arm_update()
    self.base_update()


  # cleanly stop all actions and save data

  def shutdown(self):
    print('')
    print('Shutting down ...')

    # stop reasoning and speech
    self.ai.Done(1)
    self.reco.Done()
    self.tts.Done()

    # stop components
    self.base.Stop()
    self.body.Done()                   # lights off last

    # unmute microphone  
    os.system("pactl set-source-mute @DEFAULT_SOURCE@ 0")                           
    print('Done')
  

  # ------------------------------- SPEECH -------------------------------- 

  # transfer any utterances to TTS system

  def tts_issue(self):
    msg = self.ai.Spout()
    if msg != '':
      self.tts.Say(msg)


  # feed ALIA any speech recognition results

  def reco_update(self):
    snd = self.reco.Status()
    self.ai.Hear.value = snd      
    if snd == 2:
      msg = self.reco.Heard()
      self.ai.Spin(msg)


  # -------------------------------- BODY ---------------------------------

  # adjust LEDs on main robot body and handle muting

  def body_issue(self):

    # get talking status
    mth = self.tts.Mouth()             # -1 or color (0 = black)
    if mth < 0:
      self.ai.Talk.value = 0
    else:
      self.ai.Talk.value = 1 
     
    # microphone muting
    if mth != self.mth0:
      if self.mth0 < 0:             
        os.system("pactl set-source-mute @DEFAULT_SOURCE@ 1 &") 
      elif mth < 0:                  
        os.system("pactl set-source-mute @DEFAULT_SOURCE@ 0 &")   
      self.mth0 = mth

    # set mouth color (talk flashing > listening glow)
    col = mth
    if col < 0 and self.ai.Attn.value > 1:  
      col = 0x408000                   # listening = green
    if col != self.col0:                   
      self.body.Talk(col)  
      self.col0 = col

    # modulate action speeds based on emotion
    m = self.ai.Mood.value
    self.sf = 1.0
    if m & 0x21 != 0:                  # scared or tired
      self.sf = 0.8
    if m & 0x0140 != 0:                # very happy or angry
      self.sf *= 1.2

    # set body color based on mood bits
    # [ surprised angry scared happy : unhappy bored lonely tired ]
    if m & 0x40 != 0:
      bc = 0xFF2020                    # angry = red
    elif m & 0x20 != 0:
      bc = 0xE0FF20                    # scared = pale yellow
    elif m & 0x10 != 0:
      bc = 0xFF8080                    # happy = pink             
    elif m & 0x08 != 0: 
      bc = 0x080018                    # unhappy = violet 
    else:
      bc = 0xFF8000                    # neutral = orange
    self.body.Breath(bc)
    self.body.Flash(m & 0x80)          # surprised (white)


  # get battery level from main robot

  def body_update(self):

    # piece-wise linear approximation to capacity
    v100, v20, v10, v0 = 8.0, 7.0, 6.7, 6.5
    v = self.body.Battery()
    if v >= v100:
      pct = 100.0
    elif v >= v20:
      pct = 80.0 * (v - v20) / (v100 - v20) + 20.0
    elif v >= v10:
      pct = 10.0 * (v - v10) / (v20 - v10) + 10.0
    elif v >= v0:
      pct = 10.0 * (v - v0) / (v10 - v0)
    else:
      pct = 0.0
    self.ai.Batt.value = pct

    # get overall body attitude (degs)
    self.imu.Update();
    self.ai.Tilt = self.imu.Incline()
    self.ai.Roll = self.imu.Roll()

  
  # -------------------------------- ARM ----------------------------------

  # set arm joint positions based on some target
  # also handles neck commands to aim camera

  def arm_issue(self):

    # get importance of arm and neck commands
    abid = max(self.ai.Api.value, self.ai.Adi.value)
    jbid = self.ai.Aji.value
    nbid = max(self.ai.Npi.value, self.ai.Nti.value) 

    # use arm to aim camera (pseudo-neck)
    if nbid > max(abid, jbid):
      p0, t0, _ = self.arm.View()
      pan = self.ai.Npt.value 
      if self.ai.Npv.value == 0:
        pan = p0                         # no motion default
      tilt = self.ai.Ntt.value 
      if self.ai.Ntv.value == 0:
        tilt = t0                        # no motion default
      sp = max(self.ai.Npv.value, self.ai.Ntv.value)
      self.arm.Gaze(pan, tilt, self.sf * sp)

    # return arm to tucked travel position
    elif jbid > abid:
      b, s, e, w = self.arm.Home()
      self.arm.Pose(b, s, e, w, self.sf * self.ai.Ajv.value) 

    # use arm to position gripper (check mode ...)
    else:
      x, y, z = self.ai.Axt.value, self.ai.Ayt.value, self.ai.Azt.value
      if self.ai.Apv.value == 0:
        x, y, z = self.arm.Position()    # no motion default
      t = self.ai.Att.value
      tex = self.ai.Adm.value & 0x02     # exact tilt 
      if self.ai.Adv.value == 0:
        _,t,_ = self.arm.Orientation()   # no motion default
        tex = 0
      sp = max(self.ai.Apv.value, self.ai.Adv.value)
      if self.ai.Apm.value != 0 or self.ai.Adm.value & 0x07 != 0:
        sp = -abs(sp)                    # linear trajectory 
      self.arm.Move(x, y, z, t, tex, self.sf * sp)

    # interpret hand command then set all arm joints (check sp on squeeze)
    self.arm.Grip(self.ai.Awt.value, self.sf * self.ai.Awv.value)
    self.arm.Issue()


  # get current position, orientation, and gripper width from arm

  def arm_update(self):

    # gripper position and orientation
    x, y, z = self.arm.Position()
    p, t, r = self.arm.Orientation()
    self.ai.Ax.value, self.ai.Ay.value, self.ai.Az.value = x, y, z
    self.ai.Ap.value, self.ai.At.value, self.ai.Ar.value = p, t, r

    # finger separation and force
    self.ai.Aw.value = self.arm.Width()
    self.ai.Af.value = self.arm.Squeeze()

    # deviation from tucked pose
    b, s, e, w = self.arm.Home()
    self.ai.Aj.value = self.arm.ErrAng(b, s, e, w)

    # current camera position and gaze direction
    self.ai.Nx.value, self.ai.Ny.value, self.ai.Nz.value = self.arm.Camera()
    self.ai.Np.value, self.ai.Nt.value, self.ai.Nr.value = self.arm.View()


  # -------------------------------- BASE ---------------------------------

  # set wheel velocities based on rate and sign of incremental amount  

  def base_issue(self):
    msp = self.sf * self.ai.Bmv.value
    if self.ai.Bmt.value < 0:
      msp = -msp
    tsp = self.sf * self.ai.Brv.value
    if self.ai.Brt.value < 0:
      tsp = -tsp
    self.base.Drive(msp, tsp, self.ai.Bsk.value)


  # get odometry estimate from wheels and updated body orientation

  def base_update(self):
    self.base.Compass(self.imu.Heading())        # imu.Update already called
    self.base.Update()
    self.ai.Bx.value, self.ai.By.value, self.ai.Bh.value = self.base.Odom()


# =========================================================================

# initialize robot and start reacting to speech and sensors

if __name__ == "__main__":
  g = GanbeiAct()
  g.Run()

 