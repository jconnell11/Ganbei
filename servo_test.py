#!/usr/bin/env python3
# encoding: utf-8

import sys, time

sys.path.append('/home/pi/MasterPi/HiwonderSDK')
import Board

if __name__ == "__main__":
  print("setting servos")
  Board.setPWMServoPulse(6, 1500, 1500)    # base: forward
  Board.setPWMServoPulse(5, 1500, 1500)    # shoulder: forward 45
  Board.setPWMServoPulse(4, 1500, 1500)    # elbow: flat
  Board.setPWMServoPulse(3, 1500, 1500)    # wrist: down 45
  Board.setPWMServoPulse(1, 1500, 1500)    # gripper: 23mm sep
  time.sleep(2)