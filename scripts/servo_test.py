#!/usr/bin/env python3
# encoding: utf-8

import sys, time

sys.path.append('/home/pi/Ganbei/scripts')
from mpi_hiwonder import MasterPi


if __name__ == "__main__":
  bot = MasterPi()
  bot.ZeroDevs()             # clear Hiwonder values
  bot.Eyes(0)
  bot.Body(0)
  print("setting servos")
  bot.Servo(6, 1500, 1.5)    # base: forward
  bot.Servo(5, 1500, 1.5)    # shoulder: forward 45
  bot.Servo(4, 1500, 1.5)    # elbow: flat
  bot.Servo(3, 1500, 1.5)    # wrist: down 45
  bot.Servo(1, 1500, 1.5)    # gripper: 23mm sep
  time.sleep(2)