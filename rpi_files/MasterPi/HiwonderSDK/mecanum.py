#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi/')
import math
import threading
import HiwonderSDK.Board as Board

class MecanumChassis:
    # A = 67  # mm
    # B = 59  # mm
    # WHEEL_DIAMETER = 65  # mm

    def __init__(self, a=67, b=59, wheel_diameter=65):
        self.a = a
        self.b = b
        self.wheel_diameter = wheel_diameter
        self.velocity = 0
        self.direction = 0
        self.angular_rate = 0

    def reset_motors(self):
        for i in range(1, 5):
            Board.setMotor(i, 0)
            
        self.velocity = 0
        self.direction = 0
        self.angular_rate = 0

    def set_velocity(self, velocity, direction, angular_rate, fake=False):
        """
        Use polar coordinates to control moving
        motor1 v1|  ↑  |v2 motor2
                 |     |
        motor3 v3|     |v4 motor4
        :param velocity: mm/s
        :param direction: Moving direction 0~360deg, 180deg<--- ↑ ---> 0deg
        :param angular_rate:  The speed at which the chassis rotates
        :param fake:
        :return:
        """
        rad_per_deg = math.pi / 180
        vx = velocity * math.cos(direction * rad_per_deg)
        vy = velocity * math.sin(direction * rad_per_deg)
        vp = -angular_rate * (self.a + self.b)
        v1 = int(vy + vx - vp) 
        v2 = int(vy - vx + vp)
        v3 = int(vy - vx - vp)
        v4 = int(vy + vx + vp)
        if fake:
            return
        # motors max out at 100 so make sure to maintain ratios
        top = max(abs(v1), abs(v2), abs(v3), abs(v4)) / 100.0
        if top > 1:
          v1 = int(v1 / top)
          v2 = int(v2 / top)
          v3 = int(v3 / top)
          v4 = int(v4 / top)
        Board.setMotor(1, v1) 
        Board.setMotor(2, v2)
        Board.setMotor(3, v3)
        Board.setMotor(4, v4)
        self.velocity = velocity
        self.direction = direction
        self.angular_rate = angular_rate

    def translation(self, velocity_x, velocity_y, fake=False):
        velocity = math.sqrt(velocity_x ** 2 + velocity_y ** 2)
        if velocity_x == 0:
            direction = 90 if velocity_y >= 0 else 270  # pi/2 90deg, (pi * 3) / 2  270deg
        else:
            if velocity_y == 0:
                direction = 0 if velocity_x > 0 else 180
            else:
                direction = math.atan(velocity_y / velocity_x)  # θ=arctan(y/x) (x!=0)
                direction = direction * 180 / math.pi
                if velocity_x < 0:
                    direction += 180
                else:
                    if velocity_y < 0:
                        direction += 360
        if fake:
            return velocity, direction
        else:
            return self.set_velocity(velocity, direction, 0)

