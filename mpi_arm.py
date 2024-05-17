#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# mpi_arm.py : interface to MasterPi robot 4 DOF arm and gripper
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

import time, math, os, yaml, sys

sys.path.append('/home/pi/MasterPi/HiwonderSDK')
import Board
import yaml_handle


# -------------------------------------------------------------------------

# sine of angle in degrees

def sind(ang):
  return math.sin(ang * math.pi / 180)


# cosine of angle in degrees

def cosd(ang):
  return math.cos(ang * math.pi / 180)


# arcsine in degrees of given value 

def asind(val):
  return 180 * math.asin(val) / math.pi


# arccosine in degrees of given value 

def acosd(val):
  return 180 * math.acos(val) / math.pi


# arctangent in degrees of given offsets
def atan2d(y, x):
  return 180 * math.atan2(y, x) / math.pi


# find hypotenuse and base angle for a point in rectangular coords

def dist_ang(dx, dy):
  return math.sqrt(dx * dx + dy * dy), atan2d(dy, dx)


# find interior angle C opposite side c in a-b-c triangle
#   c^2 = a^2 + b^2 - 2 * a * b * cos(C)

def oblique_eqn(a, b, c):
  return acosd((a * a + b * b - c * c) / (2 * a * b))


# find interior angle A opposite side a in a-b-c triangle
#   sin(A) / a = sin(B) / b = sin(C) / c

def sine_law(a, b_int, b):
  return asind(a * sind(b_int) / b)


# =========================================================================

# interface to MasterPi robot 4 DOF arm and gripper
# geometry is particularly simple and has a closed-form solution
#   b = base swivel  +45 = aimed left        -45 = aimed right
#   s = shoulder     +45 = tilted forward    -45 = tilted back
#   e = elbow        +45 = down wrt se link  -45 = up wrt se link
#   w = wrist        +45 = up wrt ew link    -45 = down wrt ew link
#   g = gripper      +45 = some open         -45 = closed
# runs open-loop with no actual joint angles (assumes perfect servo)
# uses speed to linearly adjust commands toward targets
# maintains:
#   current (last cmd) angles:  bc, sc, ec, wc, gc
#   next trajectory angles:     b2, s2, e2, w2, g2
#   target joint angles:        bt, st, et, wt, gt
#   current (last cmd) coords:  xc, yc, zc, tc (angle)
#   target hand coords:         xt, yt, zt, tt (angle)
# NOTE: assumes elbow servo has been rotated 45 degs (ez)

class MpiArm:

  # set parameters and initialize state
  def __init__(self):

    # hand geometry
    self.fout =  1.54                  # wrist to finger pivot (39mm)
    self.fup  = -0.04                  # tips wrt centerline (1mm)
    self.jaw  =  1.89                  # finger link length (48mm)
    self.sep  =  0.91                  # pivot separation (23mm)
    self.wmax =  3.00                  # max gripper opening
    self.gc0  = self.Grip(0)           # minimum physical gripper angle

    # arm geometry
    self.sy = 1.97                     # base axis wrt turn center (50mm)
    self.sz = 5.08                     # shoulder axis wrt floor (129mm)
    self.se = 2.28                     # shoulder to elbow (58mm)
    self.ew = 2.46                     # elbow to wrist (62.5mm)
    self.ez = -45.0                    # elbow zero angle wrt lower link

    # camera geometry
    self.cf = 2.05                     # camera ahead of wrist (52mm)
    self.cr = 1.18                     # camera above hand (30mm)

    # speeds and angle limits
    self.gps  = 120.0                  # 360 max (gripper)
    self.dps  = 120.0                  # 360 max (arm)
    self.ips  = 12.0                   # 36 max (approx)
    self.lead = 1.6                    # smoother servo response

    # calibration data
    self.servo_lims()
    self.cam_aim()

    # set starting pose/position                        
    g = 1.0                            # needs non-zero
    b = 1.0
    _, s, e, w = self.Home()
    x, y, z, t = self.fwd_kin(b, s, e, w, g)

    # initialize state variables
    self.gt, self.gc, self.g2 = g, g, g
    self.oz, self.fs = 0.0, 0.0
    self.bt, self.st, self.et, self.wt = b, s, e, w        # final target angs
    self.bc, self.sc, self.ec, self.wc = b, s, e, w        # current angs
    self.b2, self.s2, self.e2, self.w2 = b, s, e, w        # next servo angs
    self.xt, self.yt, self.zt, self.tt = x, y, z, t
    self.xc, self.yc, self.zc, self.tc = x, y, z, t
    self.sp, self.firm, self.mode = 0.0, 0, -1             # special init mode

    # send command to servos but don't wait for completion
    self.Rate(30)
    self.send_servos(b, s, e, w, g)                


  # load "Deviation.yaml" file to adjust servo angular ranges
  # empirically 1400us swing -> 123 degs so f = 0.088, rng = 176 degs

  def servo_lims(self):
    data = yaml_handle.get_yaml_data(yaml_handle.Deviation_file_path)
    bdev = -0.088 * data['6']
    self.b0, self.b1 = -88 + bdev, 88 + bdev
    sdev = -0.088 * data['5']
    self.s0, self.s1 = -88 + sdev, 88 + sdev
    edev = -0.088 * data['4']
    self.e0, self.e1 = -88 + edev, 88 + edev
    wdev = -0.088 * data['3']
    self.w0, self.w1 = -88 + wdev, 88 + wdev


  # load "cam_calib.yaml" file to adjust camera pan and tilt

  def cam_aim(self):
    path = os.path.dirname(os.path.realpath(__file__))
    try:
      with open(path + "/calib/mpi_calib.yaml", 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    except:
      data = {}
    self.cp0 = data.get('cam_p0', 0.0)
    self.ct0 = data.get('cam_t0', 0.0)
    self.cr0 = data.get('cam_r0', 0.0)


  # remember update rate in order to respect speed limits
  # sets servo time slightly long to achieve smooth overlap

  def Rate(self, hz):
    self.cyc = 1.0 / hz
    self.ms = int(1000 * self.lead * self.cyc + 0.5)  


  # -----------------------------------------------------------------------

  # tell maximum gripper width in inches

  def Open(self):
    return self.wmax         # gc = 34 degs


  # tell current gripper width in inches
  # adjust servo 1 deviation so 0 degs -> 23mm separation

  def Width(self):
    w = self.sep + 2 * self.jaw * sind(self.gc)
    return max(0.0, w)


  # tell current gripper exerted force in ounces
  # really just hysteretic: positive = holding, 0 = empty

  def Squeeze(self):
    if self.oz <= 0 and self.gc <= -20:
      self.oz = 7.0                    # 60% of 1.5 Kg-cm @ 48mm
    elif self.oz > 0 and self.gc >= 30:
      self.oz = 0.0
    return self.oz


  # set goal to shift to gripper width or force at some speed 
  # wf >= 0 means a specific width in inches, wf < 0 means apply force 
  # returns target servo angle for convenience

  def Grip(self, wf, speed =1.0):
    if wf < 0:
      self.gt = -25.0        # -14 = closed + 10 for force
    else:
      cmd = max(0.0, min(wf, 1.1 * self.Open()))
      self.gt = asind((cmd - self.sep) / (2 * self.jaw))
    self.fs = speed
    return self.gt


  # -----------------------------------------------------------------------

  # canonical home pose for arm allowing travel and camera aiming
  # returns base, shoulder, elbow, wrist in degs wrt nominal

  def Home(self):
    return 0.0, -60.0, 90.0, -15.0


  # tell servo joint angles wrt nomimal in degrees
  # returns base, shoulder, elbow, wrist in degs

  def Angles(self):
    return self.bc, self.sc, self.ec, self.wc


  # set goal to shift to joint configuration at some speed
  # base, shoulder, elbow, wrist are in degs wrt nominal
  # sp is speed as a fraction of "normal" (dps = 120)
  # NOTE: assumes elbow servo has been rotated 45 degs (ez)

  def Pose(self, bgoal, sgoal, egoal, wgoal, speed =1.0):
    self.bt, self.st, self.et, self.wt = bgoal, sgoal, egoal, wgoal
    self.sp = speed
    self.mode = 0                      # angular


  # tell max offset in any joint wrt reference pose in degrees

  def ErrAng(self, bref, sref, eref, wref):
    bdev = abs(self.bc - bref)
    sdev = abs(self.sc - sref)
    edev = abs(self.ec - eref)
    wdev = abs(self.wc - wref)
    return max(bdev, sdev, edev, wdev)


  # -----------------------------------------------------------------------

  # tell location of grip center wrt body coords in inches
  # origin is center of 4 wheels and floor (x to right, y forward)

  def Position(self):
    return self.xc, self.yc, self.zc


  # tell hand elevation wrt horizontal and pointing azimuth in degrees
  # hand pan forward = +90 degrees (0 degs = right)

  def Orientation(self):
    return self.bc + 90, self.tc, 0.0   


  # set goal to shift grip center to some position at some speed
  # x, y, z are in inches, tip is deviation from horizontal in degs
  # coordinate origin is center of 4 wheels and floor (x to right)
  # exact > 0 says preferred tilt can be modified if needed
  # sp is top speed as a fraction of normal (dps = 120, ips = 12)
  # negative sp means linear path (often slower) not angular slew

  def Move(self, xgoal, ygoal, zgoal, tgoal, exact =0, speed =1.0):  
    self.xt, self.yt, self.zt, self.tt = xgoal, ygoal, zgoal, tgoal
    self.sp, self.firm = abs(speed), exact
    if speed < 0:
      self.mode = 1                    # linear hand path
    else:
      b, s, e, w = self.inv_kin(xgoal, ygoal, zgoal, tgoal, exact, self.gt)  
      self.bt, self.st, self.et, self.wt = b, s, e, w
      self.mode = 0


  # tell distance of finger grip point from reference point in inches
  # included for local use - not need by ALIA

  def ErrPos(self, xref, yref, zref):
    dx = self.xc - xref
    dy = self.yc - yref
    dz = self.zc - zref
    return math.sqrt(dx * dx + dy * dy + dz * dz)


  # determine absolute tip error wrt reference tilt in degs
  # included for local use - not need by ALIA

  def ErrTip(self, tref):
    return abs(self.tc - tref)

  
  # -----------------------------------------------------------------------

  # tell real-world camera location in inches
  # coordinate origin is center of 4 wheels and floor (x to right)

  def Camera(self):
    x, y, z, t = self.wrist_rel(self.bc, self.sc, self.ec, self.wc, self.cf, self.cr)
    return x, y, z


  # tell camera pointing direction wrt body centerline in degs

  def View(self):
    return self.bc + self.cp0, self.tc + self.ct0, self.cr0
    

  # generates pan and tilt goals to aim camera at real-world target
  # x,y,z in inches, origin is center of 4 wheels and floor (x to right) 
  # returns pan and tilt angles based on current camera position
  # Note: as pan and tilt change so will camera position -> different aim 
  # included for local use - not need by ALIA

  def AimFor(self, x, y, z):
    p = atan2d(-x, y - self.sy)
    cx, cy, cz = self.Camera()
    dx = x - cx
    dy = y - cy
    dr = math.sqrt(dx * dx + dy * dy)
    dz = z - cz
    t = atan2d(dz, dr)
    return p, t


  # use wrist and base joints to aim camera in some direction
  # assumes angles are in degrees wrt global coords (not pixels)

  def Gaze(self, pan, tilt, speed =1.0):

    # pan can only be affected by base angle
    b = pan - self.cp0
    t = tilt - self.ct0

    # try adjusting wrist first to achieve tilt
    e = max(-45, min(self.ec, 45))
    s = max(-45, min(self.sc, 45))
    w0 = t + (e - self.ez) - (90 - s)  
    w = max(-45, min(w0, 45))
  
    # if outside limits try adjusting shoulder then elbow
    if w != w0:
      s0 = -t + w - (e - self.ez) + 90
      s = max(-45, min(s0, 45))
      if s != s0:
        e = -t + w + (90 - s) + self.ez
    self.Pose(b, s, e, w, speed)


  # aim camera at some real-world target based on its current position
  # x,y,z in inches, origin is center of 4 wheels and floor (x to right) 
  # included for local use - not need by ALIA

  def LookAt(self, x, y, z, speed =1.0):
    p, t = self.AimFor(x, y, z)
    self.Gaze(p, t, speed)


  # tell maximum angular offset (in degs) from desired pan and tilt
  # included for local use - not need by ALIA

  def ErrGaze(self, pan, tilt):
    dp = abs((self.bc + self.cp0) - pan)
    dt = abs((self.tc + self.ct0) - tilt)
    return max(dp, dt)


  # determine angular offset of gaze from some real-world target
  # x,y,z in inches, origin is center of 4 wheels and floor (x to right) 
  # included for local use - not need by ALIA

  def ErrLook(self, x, y, z):
    p, t = self.AimFor(x, y, z)
    return self.ErrGaze(p, t)


  # -----------------------------------------------------------------------      

  # send new joint angles based on targets and speeds
  # uses bt, st, et, wt, gt to produce b, s, e, w, g
  # typically called at beginning of new cycle (before reading sensors)
  # returns 1 if okay, 0 if some joint angle clipping

  def Issue(self):

    # assume perfect servos so all incremental targets achieved
    b, s, e, w, g = self.b2, self.s2, self.e2, self.w2, self.g2
    self.bc, self.sc, self.ec, self.wc, self.gc = b, s, e, w, g
    self.xc, self.yc, self.zc, self.tc = self.fwd_kin(b, s, e, w, g)

    # get new gripper trajectory point and servo command
    inc = self.fs * self.gps * self.cyc
    self.g2 = self.gc + self.v_ramp(self.gc - self.gt, inc)
    self.g2 = max(-25, min(self.g2, 40))
    g = self.gc + self.lead * (self.g2 - self.gc)

    # get new arm joint angles
    if self.mode <= 0:
      self.linear_ang() 
    else:
      self.linear_xyz()   

    # send angles to arm joints
    ok, b, s, e, w = self.joint_cmd()                    
    self.send_servos(b, s, e, w, g)
    return ok


  # linearly ramp angles in current arm pose toward target pose

  def linear_ang(self):
   
    # find signed joint errors and max magnitude
    db = self.bc - self.bt
    ds = self.sc - self.st
    de = self.ec - self.et
    dw = self.wc - self.wt    
    top = max(0.1, abs(db), abs(ds), abs(de), abs(dw))

    # adjust joint speeds so all finish at the same time
    da = self.sp * self.dps * self.cyc 
    self.b2 = self.bc + self.v_ramp(db, da * abs(db) / top)
    self.s2 = self.sc + self.v_ramp(ds, da * abs(ds) / top)
    self.e2 = self.ec + self.v_ramp(de, da * abs(de) / top)
    self.w2 = self.wc + self.v_ramp(dw, da * abs(dw) / top)


  # linearly ramp current Cartesian grip point toward target location
  # assumes target is bt, st, et, wt and current angles are bc, sc, ec, wc
  # computes incremental target pose b2, s2, e2, w2 to achieve by next cycle

  def linear_xyz(self):
   
    # find signed coordinate and tilt errors
    dx = self.xc - self.xt
    dy = self.yc - self.yt
    dz = self.zc - self.zt
    dt = self.tc - self.tt
    
    # compute max servo move and slew per cycle
    dp = self.sp * self.ips * self.cyc
    lim = 0.5 * 360 * self.cyc        

    # scale translation and rotation so they finish simultaneously
    pcyc = self.ErrPos(self.xt, self.yt, self.zt) / dp
    tcyc = abs(dt) / lim
    cnt = max(0.1, pcyc, tcyc)
    pf = pcyc / cnt
    tf = tcyc / cnt 

    # adjust coordinate speeds for direction of travel
    top = max(0.1, abs(dx), abs(dy), abs(dz))
    xf = pf * abs(dx) / top
    yf = pf * abs(dy) / top
    zf = pf * abs(dz) / top

    # step in xyzt and find corresponding shift in joint angles
    inc = dp
    ang = lim
    while True:

      # shift by balanced speed-limited amount
      x2 = self.xc + self.v_ramp(dx, inc * xf)
      y2 = self.yc + self.v_ramp(dy, inc * yf)
      z2 = self.zc + self.v_ramp(dz, inc * zf)
      t2 = self.tc + self.v_ramp(dt, ang * tf)

      # find joint angles then check if angular speed okay
      b, s, e, w = self.inv_kin(x2, y2, z2, t2, self.firm, self.g2)
      self.b2, self.s2, self.e2, self.w2 = b, s, e, w
      dev = self.ErrAng(b, s, e, w)
      if dev <= lim:
        break

      # if rotation too fast then try a smaller step   
      if inc > 0.05 or ang > 0.5:   
        inc *= 0.7                  
        ang *= 0.7 
        continue

      # give up and do joint interpolation instead
      f = lim / dev
      self.b2 = self.bc + f * (b - self.bc)
      self.s2 = self.sc + f * (s - self.sc)
      self.e2 = self.ec + f * (e - self.ec)
      self.w2 = self.wc + f * (w - self.wc)
      break


  # get amount to shift command to fix "err" but limit to "inc"
  # init mode gives increment to make current angle be exactly target

  def v_ramp(self, err, inc):
    if self.mode < 0:                  # special init mode
      return -err
    if err < 0:
      return min(inc, -err)
    return -min(inc, err)


  # limit joints to valid range and produce servo commands with overshoot
  # b2, s2, e2, w2 values are proposed trajectory point
  # returns 1 if okay, 0 if some joint angle clipping

  def joint_cmd(self):
  
    # clamp joint angles of trajectory point to valid range
    bb, ss, ee, ww = self.b2, self.s2, self.e2, self.w2
    self.b2 = max(self.b0, min(self.b2, self.b1))
    self.s2 = max(self.s0, min(self.s2, self.s1))
    self.e2 = max(self.e0, min(self.e2, self.e1))
    self.w2 = max(self.w0, min(self.w2, self.w1))

    # set command some distance beyond new point (but same speed)
    b = self.bc + self.lead * (self.b2 - self.bc)
    s = self.sc + self.lead * (self.s2 - self.sc)
    e = self.ec + self.lead * (self.e2 - self.ec)
    w = self.wc + self.lead * (self.w2 - self.wc)

    # see if any angles were changed
    if self.b2 != bb or self.s2 != ss or self.e2 != ee or self.w2 != ww:
      return 0, b, s, e, w
    return 1, b, s, e, w


  # send angle position commands to controller
  # servo command angles are self.bs, etc 
  # moves super-slow on first command

  def send_servos(self, b, s, e, w, g):
    time = self.ms if self.mode >= 0 else 1500   # init = hw ramp
    args =  [time, 5]
    args += [6, self.a2t(b)]                     # base
    args += [5, self.a2t(s)]                     # shoulder
    args += [4, self.a2t(e)]                     # elbow
    args += [3, self.a2t(w)]                     # wrist
    args += [1, self.a2t(g)]                     # gripper
    Board.setPWMServosPulse(args)


  # convert angle to pulse length (usec) for servo

  def a2t(self, ang):
    return int(11.3636 * ang + 1500 + 0.5)       # empirical


  # -----------------------------------------------------------------------

  # forward kinematics 
  # convert joint angles in degs to world position of grasp center
  # returns x, y, z, in inches and gripper tilt in degs
  # NOTE: assumes elbow servo has been rotated 45 degs (ez)

  def fwd_kin(self, b, s, e, w, g =0.0):
    wf = self.fout + self.jaw * cosd(max(self.gc0, g))
    return self.wrist_rel(b, s, e, w, wf, self.fup)


  # find location of a point with some displacement wrt gripper axis
  # "fwd" is along axis of fingers, "up" is orthogonal to that 
  # used both for fingertips (rise = 0) and for camera (rise > 0)
  # NOTE: assumes elbow servo has been rotated 45 degs (ez)

  def wrist_rel(self, b, s, e, w, fwd, rise):
    sup = 90 - s
    eup = sup - (e - self.ez)            
    wr = self.se * cosd(sup) + self.ew * cosd(eup) 
    wz = self.se * sind(sup) + self.ew * sind(eup)
    tup = eup + w   
    r = wr + fwd * cosd(tup) - rise * sind(tup) 
    z = wz + fwd * sind(tup) + rise * cosd(tup)
    x = -r * sind(b)
    y =  r * cosd(b)
    return x, y + self.sy, z + self.sz, tup


  # inverse kinematics
  # determine joint angles for given world position of grasp center
  # also takes desired hand tilt "t" in degs and whether it is fixed
  # needs gripper servo angle "g" in degs to find grasp center
  # configures arm for closest approach to goal given constraints
  # returns base, shoulder, elbow, and wrist angles in degs
  # NOTE: assumes elbow servo has been rotated 45 degs (ez)

  def inv_kin(self, x, y, z, t, exact =0, g =0.0):
    
    # find wrist to grip center given hand servo angle
    wf = self.fout + self.jaw * cosd(max(self.gc0, g))

    # cylindrical coords of goal (gr gz) wrt shoulder
    gr, b = dist_ang(y - self.sy, -x)
    gz = z - self.sz
    sg, gup = dist_ang(gr, gz)

    # if super far consider making whole arm straight (e = -45, w = 0)
    sw_max = self.se + self.ew
    if exact <= 0:
      if sg >= sw_max + wf:  
        return b, 90 - gup, self.ez, 0.0         # super far adjust

    # needed coords of wrist (wr wz) wrt shoulder for fixed tilt 
    wr = gr - wf * cosd(t) + self.fup * sind(t)
    wz = gz - wf * sind(t) - self.fup * cosd(t)
    sw, wup = dist_ang(wr, wz)

    # if too far consider making lower arm straight (e = -45)
    if sw >= sw_max:

      # find closest finger approach given fixed tilt
      if exact > 0:
        f_int = gup - t
        s_int = sine_law(wf, f_int, sw_max)
        sup = gup + s_int
        return b, 90 - sup, self.ez, t - sup     # super far fixed

      # adjust tilt to hit goal (known closer than super far)
      w_int = oblique_eqn(sw_max, wf, sg)
      w = w_int - 180
      s_int = sine_law(wf, w_int, sg)
      sup = gup + s_int
      if t > 0:              # wrist down
        w = -w                
        sup = gup - s_int        
      return b, 90 - sup, self.ez, w             # reachable adjust
   
    # find elbow bend (up) for reachable point using fixed tilt
    e_int = oblique_eqn(self.se, self.ew, sw)
    e = (180 - e_int) + self.ez
    s_int = sine_law(self.ew, e_int, sw)
    sup = wup + s_int                                   
    eup = sup - (e - self.ez)
    return b, 90 - sup, e, t - eup               # reachable fixed


# =========================================================================

# moves incrementally in x then z then back to home

if __name__ == "__main__":
  print('initializing ...')
  a = MpiArm()
  time.sleep(2)

  """
  # reconfigure to specified pose
  print('posing arm')
  a.Pose(35.0, -10.0, 130.0, 40.0, 0.5)
  for i in range(40):
    print("%2d cmd = %6.2f %6.2f %6.2f %6.2f" % (i, a.bc, a.sc, a.ec, a.wc))
    a.Issue()
    time.sleep(a.cyc)
  print('done')
  """

  """
  # test forward and inverse kinematics
  print("forward")
  g = 30.0
  b, s, e, w = -20.0, -45.0, 90.0, 35.0
  print("  bsew = %3.1f : (%3.1f %3.1f %3.1f), g = %3.1f" % (b, s, e, w, g))
  x, y, z, t = a.fwd_kin(b, s, e, w, g)
  print("  --> xyz = (%4.2f %4.2f %4.2f), tilt = %4.2f" % (x, y, z, t))
 
  print("normal inverse")
  f = 1
  print("  xyz = (%3.1f %3.1f %3.1f), tilt = %3.1f, ex = %d, g = %3.1f" % (x, y, z, t, f, g))
  b, s, e, w = a.inv_kin(x, y, z, t, f, g)
  print("  --> bsew = %4.2f : (%4.2f %4.2f %4.2f)" % (b, s, e, w))

  print("super far adjust")
  f = 0
  x, y, z = -8.49, 10.46, 17.08
  print("  xyz = (%3.1f %3.1f %3.1f), tilt = %3.1f, ex = %d, g = %3.1f" % (x, y, z, t, f, g))
  b, s, e, w = a.inv_kin(x, y, z, t, f, g)
  print("  --> bsew = %4.2f : (%4.2f %4.2f %4.2f)" % (b, s, e, w))

  print("super far fixed")
  f = 1
  x, y, z, t = 0.0, 13.97, 17.08, 5.0
  print("  xyz = (%3.1f %3.1f %3.1f), tilt = %3.1f, ex = %d, g = %3.1f" % (x, y, z, t, f, g))
  b, s, e, w = a.inv_kin(x, y, z, t, f, g)
  print("  --> bsew = %4.2f : (%4.2f %4.2f %4.2f)" % (b, s, e, w))

  print("reach adjust")
  f = 0
  x, y, z, t = 0.0, 8.50, 8.43, -20.0
  print("  xyz = (%3.1f %3.1f %3.1f), tilt = %3.1f, ex = %d, g = %3.1f" % (x, y, z, t, f, g))
  b, s, e, w = a.inv_kin(x, y, z, t, f, g)
  print("  --> bsew = %4.2f : (%4.2f %4.2f %4.2f)" % (b, s, e, w))
  """

  """
  # reconfigure to specified position
  # reachable
#  x, y, z, t, f, sp = -4, 5, 9, 0, 1, -1
  x, y, z, t, f, sp = -4, 6, 3, -60, 1, -1    
#  x, y, z, t, f, sp = 5, 5, 7, -30, 1, -1
#  x, y, z, t, f, sp = 0, 5, 2, -90, 1, -1
  # super far
#  x, y, z, t, f, sp = 0, 12, 8, -30, 1, -1   
#  x, y, z, t, f, sp = 0, 12, 8, 0, 1, -1    
#  x, y, z, t, f, sp = 0, 12, 8, 30, 1, -1  
  # reachable adjust
#  x, y, z, t, f, sp = 0, 9, 6, -45, 0, -1 
#  x, y, z, t, f, sp = 0, 9.5, 7.8, 60, 0, -1 
  a.Move(x, y, z, t, f, sp)
  a.bt, a.st, a.et, a.wt = a.inv_kin(a.xt, a.yt, a.zt, a.tt, a.firm, a.gt)
  print("  ang0 = %6.2f : (%6.2f %6.2f %6.2f)" % (a.bc, a.sc, a.ec, a.wc))
  print("config = %6.2f : (%6.2f %6.2f %6.2f)" % (a.bt, a.st, a.et, a.wt))
  print("  loc0 = (%6.2f %6.2f %6.2f) x %6.2f" % (a.xc, a.yc, a.zc, a.tc))
  print("target = (%6.2f %6.2f %6.2f) x %6.2f" % (a.xt, a.yt, a.zt, a.tt))
  dx, dy, dz = a.xc - a.xt, a.yc - a.yt, a.zc - a.zt
  print("")
  print("Moving arm %3.1f in..." % (math.sqrt(dx * dx + dy * dy + dz * dz)))
  x0, y0, z0, t0 = a.xc, a.yc, a.zc, a.tc
  for i in range(100): 
#    print("")
#    print("%2d loc = (%6.2f %6.2f %6.2f) x %6.2f" % (i, a.xc, a.yc, a.zc, a.tc))
#    print("%2d chg = (%5.2f %5.2f %5.2f) x %5.2f" % (i, a.xc - x0, a.yc - y0, a.zc - z0, a.tc - t0)) 
#    print("%2d ang = %6.2f : (%6.2f %6.2f %6.2f)" % (i, a.bc, a.sc, a.ec, a.wc))
    print("%2d off = (%5.1f %5.1f %5.1f) x %5.1f" % (i, a.xc - a.xt, a.yc - a.yt, a.zc - a.zt, a.tc - a.tt))
    x0, y0, z0, t0 = a.xc, a.yc, a.zc, a.tc
    if a.Issue() <= 0:
      break
    time.sleep(a.cyc)
  print('done')
  """

  """
  # look in some direction 
#  p, t = 60.0, 60.0
  p, t = 0.0, -90.0
  for i in range(100):
    err = a.ErrGaze(p, t)
    print("  (%3.1f %3.1f) -> err %3.1f degs" % (a.Pan(), a.Tilt(), err))
    if err < 1:
      break
    a.Gaze(p, t)
    a.Issue()
    time.sleep(a.cyc)
  print('done')
  """

  """
  # look at some point
  x, y, z = 6, 12, 2.7
#  x, y, z = 6, 12, 9.6
  for i in range(100):
    err = a.ErrLook(x, y, z)
    print("  err %3.1f degs" % (err))
    if err < 1:
      break
    a.LookAt(x, y, z)
    a.Issue()
    time.sleep(a.cyc)
  print('done')
  """

  # ----------------------- active demo ------------------------------
      
  # move hand diagonally by 4.5" and open hand
  print("\nmoving diagonally left")
  time.sleep(0.5)
  gt = a.Open()
  xt, yt, zt = a.Position()
  xt -= 4
  yt += 2
  a.Move(xt, yt, zt, a.Tilt(), 1, -1)
  a.Grip(gt)
  for i in range(100):
    err = a.ErrPos(xt, yt, zt)
    print("  (%3.1f %3.1f %3.1f) -> err %4.2f in" % (a.xc, a.yc, a.zc, err))
    if err < 0.1:
      break
    a.Issue()
    time.sleep(a.cyc)
  
  # move down by 3" and close gripper
  print("\ndescending and closing")
  time.sleep(0.5)
  xt, yt, zt = a.Position()
  zt -= 3
  a.Move(xt, yt, zt, a.Tilt(), 1, -1)
  a.Grip(0)
  for i in range(100):
    err = a.ErrPos(xt, yt, zt)
    print("  (%3.1f %3.1f %3.1f) -> err %4.2f in" % (a.xc, a.yc, a.zc, err))
    if err < 0.1:
      break
    a.Issue()
    time.sleep(a.cyc)

  # return to travel position
  print("\ntucking arm")
  time.sleep(0.5)
  bt, st, et, wt = a.Home()
  a.Pose(bt, st, et, wt)
  for i in range(100):
    err = a.ErrAng(bt, st, et, wt)
    print("  (%3.1f %3.1f %3.1f %3.1f) -> err %3.1f degs" % (a.bc, a.sc, a.ec, a.wc, err))
    if err < 1:
      break
    a.Issue()
    time.sleep(a.cyc)
  
  # tell final position
  x, y, z = a.Position()
  p, t, r = a.Orientation()
  print("pos (%4.2f %4.2f %4.2f), dir (%4.2f %4.2f %4.2f)" % (x, y, z, p, t, r))
  print('done')
 
