#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# Ganbei_vis.py : control MasterPi robot with ALIA reasoner and cameras
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
#
# Copyright 2025-2026 Etaoin Systems
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

import multiprocessing, termios
import math, time, os, socket, sys, signal, cv2
import numpy as np

sys.path.append('/home/pi/Ganbei/scripts')
from alia_vis import AliaVis
from azure_reco import AzureReco
from mpi_spout import MpiFace
from mpi_hiwonder import MasterPi, PlaySFX, LowBatt       

from mpi_shell import MpiShell
from mpi_arm import MpiArm
from mpi_base import MpiBase
from mpi_cam import MpiCam
from tof_cam import TofCam


# -------------------------------------------------------------------------

# control MasterPi robot with ALIA reasoner and cameras
# to show debugging images do: "python Ganbei_vis.py 2"

class GanbeiVis:

  # initialize state (takes robot interface object as argument)
  def __init__(self, mpi):
    self.bot = mpi

    # allow for graceful exit on ^C or ^Z
    signal.signal(signal.SIGINT, self.Quit)
    signal.signal(signal.SIGTERM, self.Quit)

    # reinitialize sound system early on (slow to reboot)
    os.system("pulseaudio --start > /dev/null 2>&1")

    # main loop timing (30Hz)
    self.tick = 0.0
    self.cycle = 1.0 / 30.0
    self.ok = -4

    # get beeping threshold
    self.v10 = LowBatt()

    # turn on body LEDs very early
    self.body = MpiShell(self.bot)

    # set up color camera and depth sensor
    self.rgb = MpiCam()
    self.tof = TofCam()
    self.rcnt = 0
    self.ccnt = 0
    self.rt0 = 0.0
    self.ct0 = 0.0
    self.t0  = 0.0

    # create reasoner and language components
    self.ai   = AliaVis()
    self.reco = AzureReco()
    self.face = MpiFace()

    # interface to hardware components (arm poses itself)
    self.arm  = MpiArm(self.bot)
    self.base = MpiBase(self.bot)

    # mouth LED state variables
    self.mth0 = -1
    self.col0 = -1

    # action speed factor
    self.sf = 1.0


  # request termination after next loop finishes
  # Note: can be called externally

  def Quit(self, *args):
    self.loop = False


  # debugging routine prints wifi signal strength (db > -70)

  def Wifi(self):
    os.system("echo -n 'wifi = ';iw wlan0 station dump | grep signal | awk '{print $2}'")


  # ----------------------------- MAIN LOOP ------------------------------- 

  # update sensors, reason a bit, then issue commands
  # Note: meant to be called externally

  def Run(self):
    try:
      self.start()
      while self.loop: 
        self.update()    
        if self.ai.Think() <= 0:
          break
        self.issue()
        self.pace()
    except:
      print("\n\x1b[1;33m>>> Unexpected exit!\x1b[0m")   
    self.shutdown()


  # sleep until next cycle start time

  def pace(self):
    now = time.time()
    if self.tick == 0.0:
      self.tick = now
    wait = self.tick - now
    if wait > 0:
      time.sleep(wait)   
    else:
      self.tick = now                  # fallen behind! 
    self.tick += self.cycle  


  # configure and start up all components

  def start(self):
    
    # enable microphone then set system failure flag
    print('Ganbei_vis - Initializing ...') 
    os.system("pactl set-source-mute @DEFAULT_SOURCE@ 0")  
    self.loop = False

    # whether to show debugging images 
    self.show = 0                              
    if len(sys.argv) > 1:
      if sys.argv[1].isdigit():
        self.show = int(sys.argv[1])
      else:
        print("\x1b[1;33m>>> Bad argument: show debugging images (1-14)\x1b[0m")

    # start color camera (try power-cycling if balky)
    self.ok = -4
    if self.rgb.Start() <= 0:
      print("\x1b[1;33m>>> Power cycling USB hub for color camera ... \x1b[0m", end='', flush=True)
      hub = 2 if self.v10 < 6.5 else 3     # Pi4 vs Pi5
      os.system(f"sudo uhubctl -l{hub} -a0 > /dev/null")
      time.sleep(1.0)  
      os.system(f"sudo uhubctl -l{hub} -a1 > /dev/null")
      time.sleep(3.0)                 
      os.system("pulseaudio -k")           # make sure USB sound card found 
      print()
      if self.rgb.Start() <= 0:
        print("\x1b[1;33m>>> Could not connect to color camera!\x1b[0m")
        return

    # start depth finder
    self.ok = -3
    if self.tof.Start() <= 0:
      print("\x1b[1;33m>>> Could not connect to TOF sensor!\x1b[0m")
      return

    # start speech output
    self.ok = -2
    if self.face.Start("/home/pi/Ganbei") <= 0:
      print("\x1b[1;33m>>> No text-to-speech!\x1b[0m")
      return

    # start reasoner (make name list and set image sizes)
    self.ok = -1
    if self.ai.Reset('Ganbei_vis', self.show) <= 0:      
      print("\x1b[1;33m>>> Problem with ALIA!\x1b[0m")
      return

    # start network speech recognition (needs name list)
    self.ok = 0
    if self.reco.Start() <= 0:        
      print("\n\x1b[1;33m>>> No speech recognition!\x1b[0m")
      PlaySFX("toot")                  # continue as text-only

    # possibly add camera and debugging displays (needs image sizes)
    if self.show > 0:
      self.show_init()

    # initialize video stats and watchdogs
    self.t0 = time.time()
    self.rt0 = self.t0
    self.ct0 = self.t0
    self.ok = 1
    self.loop = True                   # okay to run


  # create and display debugging image windows (after ALIA init)

  def show_init(self):

    # object detection (color camera)
    self.cam = np.zeros((480, 640, 3), np.uint8)
    self.cam[:, :, 0] = 255                                # blue
    cv2.namedWindow("Camera View", flags=cv2.WINDOW_GUI_NORMAL)
    cv2.resizeWindow("Camera View", 640, 480)
    cv2.moveWindow("Camera View", 0, 0)
    cv2.imshow("Camera View", self.cam)

    # debugging image (ALIA sets size)
    mw = self.ai.MapW()
    mh = self.ai.MapH()
    self.map = np.zeros((mh, mw, 3), np.uint8)
    self.map[:, :, 1] = 128                                # green
    cv2.namedWindow("Overhead Map", flags=cv2.WINDOW_GUI_NORMAL)
    cv2.resizeWindow("Overhead Map", mw, mh)     
    cv2.setWindowTitle("Overhead Map", self.ai.MapT())
    cv2.moveWindow("Overhead Map", 650, 0)                 # ignores!   
    cv2.imshow("Overhead Map", self.map)

    # connect images to ALIA
    self.ai.View.value = self.cam.ctypes.data 
    self.ai.Map.value  = self.map.ctypes.data
    self.ai.Vfmt.value = 2
    self.ai.Mfmt.value = 2

    # needs >200ms for fill and initialization 
    # to regrab terminal needs: sudo apt install wmctrl
    cv2.waitKey(500);   
    cv2.moveWindow("Overhead Map", 650, 0)                 # works here
    cv2.moveWindow("Camera View", 0, 0)
    title = os.getlogin() + "@" + socket.gethostname()
    os.system("wmctrl -a " + title)                        # reclaim keyboard
    

  # cleanly stop all actions and save data

  def shutdown(self):
    end = time.time()
    print("\n\nGanbei_vis - Shutting down ...")

    # stop reasoning and robot motion
    if self.ok >= 0:
      self.ai.Done(1)
    self.bot.Freeze()

    # stop speech elements
    self.reco.Done()
    self.face.Done()

    # stop color camera and depth finder
    self.rgb.Done()
    self.tof.Done()

    # lights off last
    self.body.Done()                      
   
    # get streaming stats
    if self.t0 > 0.0:
      dt = end - self.t0
      print("Range = %3.1f fps, Color = %3.1f fps" % (self.rcnt / dt, self.ccnt / dt))

    # unmute microphone  
    cv2.destroyAllWindows() 
    os.system("pactl set-source-mute @DEFAULT_SOURCE@ 0")                           
    print("Ganbei_vis - Done (%4.2fV)" % (self.body.Battery()))

    # signal that something went wrong
    if self.ok <= 0:                          
      PlaySFX("squawk")
    if self.ok == -3:
      print("\x1b[1;31m*** REBOOT TO FIX TOF SENSOR ***\x1b[0m")
      if self.show <= 0:
        os.system("sudo reboot")   
                                        

  # transfer commands from ALIA reasoner to actuators

  def issue(self):
    self.tts_issue()
    self.body_issue()
    if self.arm_mode() > 0:
      self.neck_issue()
    else:
      self.arm_issue()
    self.base_issue()
    self.img_issue()


  # get data from sensors and transfer to ALIA reasoner

  def update(self):
    self.reco_update()
    self.body_update()
    self.neck_update()
    self.arm_update()
    self.base_update()
    self.img_update()


  # determine if arm is in regular (0) or pseudo-neck mode (1)

  def arm_mode(self):
    arm = max(self.ai.Api.value, self.ai.Adi.value, self.ai.Aji.value)
    rng = max(self.ai.Rpi.value, self.ai.Rti.value, self.ai.Rgi.value)
    col = max(self.ai.Cpi.value, self.ai.Cti.value) 
    if max(rng, col) > arm:
      return 1
    return 0


  # ------------------------------- SPEECH -------------------------------- 

  # transfer any utterances to TTS system

  def tts_issue(self):
    # set appropriate prosody (and possibly expression) for current mood
    # [ surprised angry scared happy : unhappy bored lonely tired ]
    self.face.Mood(self.ai.Mood.value)

    # check for message to speak
    msg = self.ai.Spout()
    if msg != '':
      self.face.Say(msg)


  # feed ALIA any speech recognition results

  def reco_update(self):
    snd = self.reco.Status()
    self.ai.Hear.value = snd      
    if snd == 2:
      msg = self.reco.Heard()
      self.ai.Spin(msg, self.reco.Delay())


  # -------------------------------- BODY ---------------------------------

  # adjust LEDs on main robot body and handle muting

  def body_issue(self):

    # get talking status
    mth = self.face.Mouth()            # none = -1, on = 1, off = 0
    if mth < 0:
      self.ai.Talk.value = 0
    else:
      self.ai.Talk.value = 1 
    if mth > 0:
      mth = 0xFFFFFF                   # convert to RGB value for LEDs 
     
    # microphone muting
    if mth != self.mth0:
      if self.mth0 < 0:             
        os.system("pactl set-source-mute @DEFAULT_SOURCE@ 1 &") 
      elif mth < 0:                  
        os.system("pactl set-source-mute @DEFAULT_SOURCE@ 0 &")   
      self.mth0 = mth

    # set LED mouth color (listening glow < talk flashing)
    attn = self.ai.Attn.value
    self.face.Stare(attn)              # screen face eye color
    col = mth
    if col < 0 and attn > 1:  
      col = 0x80FF00                   # listening = green
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

    # set sonar color based on mood bits
    # [ surprised angry scared happy : unhappy bored lonely tired ]
    if m & 0x40 != 0:
      sc = 0xFF2020                    # angry = red
    elif m & 0x20 != 0:
      sc = 0xE0FF20                    # scared = pale yellow
    elif m & 0x10 != 0:
      sc = 0xFF0080                    # happy = magenta        
    elif m & 0x04 != 0:
      sc = 0x100018                    # bored = violet     
    elif m & 0x08 != 0: 
      sc = 0x000080                    # unhappy = blue
    else:
      sc = 0xFF8000                    # neutral = orange
    self.body.Breath(sc)
    self.body.Flash(m & 0x80)          # surprised (white)


  # get battery level from main robot

  def body_update(self):

    # piece-wise linear approximation to capacity
    v100 = 7.6
    v20  = self.v10 + 0.1                         
    v0   = v20 - 0.6  
    v = self.body.Battery()
    if v >= v100:
      pct = 100.0
    elif v >= v20:
      pct = 80.0 * (v - v20) / (v100 - v20) + 20.0
    elif v >= v0:
      pct = 20.0 * (v - v0) / (v20 - v0)
    else:
      pct = 0.0
    self.ai.Batt.value = pct

  
  # -------------------------------- NECK ---------------------------------

  # set arm joint angles based on some camera aiming specification
  # arbitration with hand pose commands happens in main issue()

  def neck_issue(self):

    # determine sensor importance
    gbid = self.ai.Rgi.value
    rbid = max(self.ai.Rpi.value, self.ai.Rti.value) 
    cbid = max(self.ai.Cpi.value, self.ai.Cti.value) 

    # color camera angles 
    if cbid > max(rbid, gbid):
      p0, t0, _ = self.arm.View()
      pan = self.ai.Cpt.value 
      if self.ai.Cpv.value == 0:
        pan = p0                       # no motion default
      tilt = self.ai.Ctt.value 
      if self.ai.Ctv.value == 0:
        tilt = t0                      # no motion default
      sp = max(self.ai.Cpv.value, self.ai.Ctv.value)
      self.arm.Gaze(pan, tilt, self.sf * sp)
      self.face.Gaze(pan, tilt, self.sf * sp * 120.0)      # needs dps

    # range-finder angles 
    elif rbid >= gbid:
      p0, t0, _ = self.arm.View()
      pan = self.ai.Rpt.value 
      if self.ai.Rpv.value == 0:
        pan = p0                       # no motion default
      tilt = self.ai.Rtt.value 
      if self.ai.Rtv.value == 0:
        tilt = t0                      # no motion default
      sp = max(self.ai.Rpv.value, self.ai.Rtv.value)
      self.arm.Gaze(pan, tilt, self.sf * sp)
      self.face.Gaze(pan, tilt, self.sf * sp * 120.0)      # needs dps

    # range-finder view location (xyz)
    else:                              # bid guaranteed non-zero
      x = self.ai.Rxt.value
      y = self.ai.Ryt.value
      z = self.ai.Rzt.value
      sp = self.ai.Rgv.value
      self.arm.LookAt(x, y, z, 2, self.sf * sp)            # avg tof + rgb

      # approx pan and tilt from screen face center (+20 deg tilt?)
      dy = y - 3.5
      dz = z - 2.5
      r = math.sqrt(x * x + dy * dy)
      pan  = math.degrees(math.atan2(-x, dy))
      tilt = math.degrees(math.atan2(dz,  r))
      self.face.Gaze(pan, tilt, self.sf * sp * 120.0)      # needs dps
     
    # ALWAYS interpret gripper command then set all arm joints 
    sp = self.ai.Awv.value if self.ai.Awi.value > 0 else 0.0
    self.arm.Grip(self.ai.Awt.value, self.sf * sp)
    self.arm.Issue()


  # get current position and orientation of sensors from arm

  def neck_update(self):

    # current depth sensor position and gaze direction
    self.ai.Rx.value, self.ai.Ry.value, self.ai.Rz.value = self.arm.Sensor(0)
    self.ai.Rp.value, self.ai.Rt.value, self.ai.Rr.value = self.arm.View()
    
    # current color camera position and gaze direction
    self.ai.Cx.value, self.ai.Cy.value, self.ai.Cz.value = self.arm.Sensor(1)
    self.ai.Cp.value, self.ai.Ct.value, self.ai.Cr.value = self.arm.View()


  # -------------------------------- ARM ----------------------------------

  # set arm joint angles based on some desired hand pose 
  # arbitration with neck commands happens in main issue()

  def arm_issue(self):

    # return arm to tucked travel position
    mbid = max(self.ai.Api.value, self.ai.Adi.value)
    if self.ai.Aji.value > mbid:
      b, s, e, w = self.arm.Home()
      b0, _, _, _ = self.arm.Angles()
      if self.arm.ErrAng(b0, s, e, w) > 2:       # swivel last
        b = b0
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
      sp = max(self.ai.Apv.value, self.ai.Adv.value) if mbid > 0 else 0.0
#      if self.ai.Apm.value != 0 or self.ai.Adm.value & 0x07 != 0:
#        sp = -abs(sp)                    # linear trajectory 
      self.arm.Move(x, y, z, t, tex, self.sf * sp)

    # ALWAYS interpret gripper command then set all arm joints 
    sp = self.ai.Awv.value if self.ai.Awi.value > 0 else 0.0
    self.arm.Grip(self.ai.Awt.value, self.sf * sp)
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


  # -------------------------------- BASE ---------------------------------

  # set wheel velocities based on rate and sign of incremental amount  

  def base_issue(self):
    msp = self.sf * self.ai.Bmv.value
    tsp = self.sf * self.ai.Brv.value
    self.base.Drive(self.ai.Bmt.value, self.ai.Brt.value, msp, tsp, self.ai.Bsk.value)


  # get odometry estimate from wheels and updated body orientation

  def base_update(self):
    self.base.Update()
    self.ai.Bt.value, self.ai.Bw.value, self.ai.Bx.value, self.ai.By.value = self.base.Odom()


  # ------------------------------- IMAGES -------------------------------- 
 
  # get new marked-up camera view and overhead map (if desired)

  def img_issue(self):
    if self.show <= 0:
      return
    cv2.imshow("Camera View", self.cam);  
    cv2.imshow("Overhead Map", self.map);       
    cv2.waitKey(1)                               # needed to pump update message


  # check for new color or range images

  def img_update(self):
    now = time.time()

    # color camera 
    buf = self.rgb.Color(0, 0)
    if buf is not None:
      self.ai.Col.value = buf
      self.ai.Cfmt.value = 3
      self.ct0 = now                             # reset timeout
      self.ccnt += 1
      if self.ok == -4:                          # restart successful
        self.ok = 1
    elif (now - self.ct0) > 0.5: 
      if self.ok <= 0:     
        self.loop = False
      else:                                      # attempt recovery   
        print("\n\x1b[1;33m>>> Color camera restarting ...\x1b[0m")
        self.ok = 0
        self.rgb.Done()
        if self.rgb.Start() > 0:
          self.ct0 = time.time()
        else:
          print("\x1b[1;33m>>> Color camera stopped working!\x1b[0m")
          self.loop = False

    # range finder
    buf = self.tof.Range(0, 0)
    if buf is not None:
      self.ai.Rng.value = buf
      self.ai.Rfmt.value = 3   
      self.rt0 = now                             # reset timeout
      self.rcnt += 1   
    elif self.ok > 0 and (now - self.rt0) > 0.5:             
      print("\n\x1b[1;33m>>> TOF sensor stopped working!\x1b[0m")
      self.ok = 0
      self.loop = False
            

# =========================================================================

# initialize robot and start reacting to speech and sensors
# "bot" must be made here so only one process owns serial port

def main():
  bot = MasterPi()    
  g = GanbeiVis(bot)
  g.Run()


# actual Python entry point

if __name__ == "__main__":

  # run main program in a separate process
  proc = multiprocessing.Process(target=main)
  proc.start()
  proc.join()

  # catch segmentation faults (SIGSEGV)
  rc = proc.exitcode
  proc.close()
  if rc != 0:

    # complain
    print("\n\x1b[1;31m>>> CRASHED!\x1b[0m")
    PlaySFX("squawk")
   
    # restore console echo
    fd = sys.stdin.fileno()
    att = termios.tcgetattr(fd);
    att[3] |= termios.ICANON | termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, att)


