#!/usr/bin/env pybricks-micropython

# ---- Comment out when running on real robot ----
#import sys, math, pathlib, os
#sys.path.insert(0, str((pathlib.Path(os.getcwd())).parent))
##from pybricks.robot import RobotSingleton as _Robot
# ------------------------------------------------

import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Button, Stop
from pybricks.tools import wait
class MyController:
    def __init__(self):
        # ---- Comment out when running on real robot ----
        #self.robot = _Robot().get_robot()
        #self.time_step = _Robot().get_time_step()
        # ------------------------------------------------

        self.ev3 = EV3Brick()
        self.left_motor = Motor(Port.B)
        self.right_motor = Motor(Port.C)
        self.ultrasonic = UltrasonicSensor(Port.S4)
        self.touch = TouchSensor(Port.S3)

        self.state = "WAITING_START"
        self.distance_traveled = 0.0
        self.reverse_distance = 0.0
        self.prev_left = None
        self.prev_right = None
        self.x, self.y, self.theta = 0.0, 0.0, 0.0

    def compute_position(self, delta_left, delta_right):
        WHEEL_RADIUS = 0.027
        WHEEL_BASE = 0.0905

        d_left = math.radians(delta_left) * WHEEL_RADIUS
        d_right = math.radians(delta_right) * WHEEL_RADIUS
        d_center = (d_left + d_right) / 2.0

        self.theta += (d_right - d_left) / WHEEL_BASE
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        return d_center

    def run(self):
        SPEED = 200
        while True:
            wait(10)
            curr_left = self.left_motor.angle()
            curr_right = self.right_motor.angle()

            if curr_left is None or curr_right is None:
                continue

            if self.prev_left is None or self.prev_right is None:
                self.prev_left = curr_left
                self.prev_right = curr_right
                continue

            delta_left = curr_left - self.prev_left
            delta_right = curr_right - self.prev_right
            self.prev_left = curr_left
            self.prev_right = curr_right

            d = self.compute_position(delta_left, delta_right)

            if self.state == "WAITING_START":
                self.left_motor.stop()
                self.right_motor.stop()
                if Button.CENTER in self.ev3.buttons.pressed():
                    self.distance_traveled = 0.0
                    self.prev_left = self.left_motor.angle()
                    self.prev_right = self.right_motor.angle()
                    self.state = "DRIVE"

            elif self.state == "DRIVE":
                self.distance_traveled += d
                self.left_motor.run(SPEED)
                self.right_motor.run(SPEED)
                
                if self.distance_traveled >= 1.185:
                    self.left_motor.brake()
                    self.right_motor.brake()
                    self.ev3.speaker.beep()
                    self.state = "WAITING"

            elif self.state == "WAITING":
                self.left_motor.hold()
                self.right_motor.hold()
                if Button.CENTER in self.ev3.buttons.pressed():
                    self.state = "DRIVE_TO_WALL"

            elif self.state == "DRIVE_TO_WALL":
                self.left_motor.run(SPEED)
                self.right_motor.run(SPEED)
               # print("Touch:", self.touch.pressed())
                if self.touch.pressed():
                    self.left_motor.stop()
                    self.right_motor.stop()
                    self.reverse_distance = 0.0
                    self.state = "REVERSE"

            elif self.state == "REVERSE":
                self.left_motor.run(-SPEED)
                self.right_motor.run(-SPEED)
                self.reverse_distance += abs(d)
                if self.reverse_distance >= 0.5:
                    self.left_motor.hold()
                    self.right_motor.hold()
                    self.ev3.speaker.beep()
                    self.state = "DONE"

            elif self.state == "DONE":
                self.left_motor.stop()
                self.right_motor.stop()
def main():
    lab1 = MyController()
    lab1.run()
if __name__ == "__main__":
    main()