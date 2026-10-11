"""Caitlin Estrada (730661469) and Madeline Clark (PID)."""

#!/usr/bin/env pybricks-micropython
import math 
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class MyController:
    def __init__(self):
        # TODO: Add team members' names and PIDs at the top of main.py.

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

        # TODO: Add variables for ultrasonic ranging.
        # - Target distance from wall: 40 cm
        # - Account for ultrasonic sensor mounting offset.
        # - Account for robot braking distance.

        # TODO: Add button press tracking.
        # Detect new presses rather than a continuously held button,
        # so one press cannot accidentally trigger multiple stages.

    def compute_position(self, delta_left, delta_right):
        # TODO: Measure the actual wheel radius and wheelbase
        # after constructing the physical robot.
        WHEEL_RADIUS = 0.027
        WHEEL_BASE = 0.0905

        d_left = math.radians(delta_left) * WHEEL_RADIUS
        d_right = math.radians(delta_right) * WHEEL_RADIUS
        d_center = (d_left + d_right) / 2.0

        self.theta += (d_right - d_left) / WHEEL_BASE
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # TODO: Consider using midpoint heading when updating x/y
        # for more accurate differential-drive odometry.

        # TODO: Consider using estimated heading to correct
        # lateral drift while driving straight.
        # Encoder odometry alone cannot detect wheel slip.

        return d_center

    def run(self):
        SPEED = 200

        # TODO: Calibrate SPEED on the physical robot.
        # Consider using a slower speed near the wall
        # to improve stopping accuracy.

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

            # OBJECTIVE 1: ODOMETRY (1.4 METERS)

            if self.state == "WAITING_START":
                self.left_motor.stop()
                self.right_motor.stop()

                # TODO: Require a fresh center-button press.

                if Button.CENTER in self.ev3.buttons.pressed():
                    self.distance_traveled = 0.0
                    self.prev_left = self.left_motor.angle()
                    self.prev_right = self.right_motor.angle()
                    self.state = "DRIVE"

            elif self.state == "DRIVE":
                self.distance_traveled += d

                self.left_motor.run(SPEED)
                self.right_motor.run(SPEED)

                # TODO: Test straight-line accuracy.
                # Adjust individual motor speeds if the robot
                # consistently drifts left or right.

                # TODO: Calibrate the 1.4 m threshold to account
                # for wheel dimensions, slip, and braking distance.

                if self.distance_traveled >= 1.4:
                    self.left_motor.brake()
                    self.right_motor.brake()
                    self.ev3.speaker.beep()
                    self.state = "WAITING"

            # OBJECTIVE 2: RANGING (40 CM FROM WALL)

            elif self.state == "WAITING":
                self.left_motor.hold()
                self.right_motor.hold()

                # TODO: Require a fresh second button press.

                if Button.CENTER in self.ev3.buttons.pressed():
                    # TODO: Change transition to RANGING.
                    # The robot must first stop 40 cm from
                    # the wall BEFORE attempting to bump it.
                    self.state = "DRIVE_TO_WALL"

            # TODO: ADD NEW STATE "RANGING"
            #
            # elif self.state == "RANGING":
            #
            # 1. Read distance using self.ultrasonic.distance().
            #    Note: UltrasonicSensor.distance() returns mm.
            #
            # 2. Drive toward the wall while monitoring distance.
            #
            # 3. Stop when the nearest point on the robot
            #    is 40 cm (400 mm) from the wall.
            #
            # 4. Account for the offset between the ultrasonic
            #    sensor and the frontmost point of the robot.
            #
            # 5. Account for stopping distance and sensor noise.
            #
            # 6. Brake, beep, and wait for the next button press.
            #
            # 7. Transition to WAITING_FOR_BUMP.

            # TODO: ADD NEW STATE "WAITING_FOR_BUMP"
            #
            # elif self.state == "WAITING_FOR_BUMP":
            #
            # 1. Keep both motors stationary.
            # 2. Wait for a fresh third center-button press.
            # 3. Transition to DRIVE_TO_WALL.

            # OBJECTIVE 3: BUMPING AND REVERSING

            elif self.state == "DRIVE_TO_WALL":
                self.left_motor.run(SPEED)
                self.right_motor.run(SPEED)

                # TODO: Consider lowering speed while approaching
                # the wall to avoid a hard collision.

                # TODO: Ensure touch sensor is mounted on the
                # frontmost surface and reliably activates.

                if self.touch.pressed():
                    self.left_motor.stop()
                    self.right_motor.stop()
                    self.reverse_distance = 0.0
                    self.state = "REVERSE"

            elif self.state == "REVERSE":
                self.left_motor.run(-SPEED)
                self.right_motor.run(-SPEED)
                self.reverse_distance += abs(d)

                # TODO: Replace or calibrate fixed-distance reversal.
                #
                # The robot must END 40 cm from the wall,
                # measured from its nearest point.
                #
                # Reversing 0.4 m does NOT necessarily mean the
                # robot ends 0.4 m from the wall.
                #
                # Suggested approach:
                # 1. Read the ultrasonic sensor while reversing.
                # 2. Convert sensor distance into distance from
                #    the nearest point on the robot.
                # 3. Stop when the adjusted distance reaches 40 cm.
                # 4. Account for sensor offset and motor braking.
                #
                # Alternative: Calibrate an encoder-based reverse
                # distance based on the robot's physical dimensions.

                if self.reverse_distance >= 0.4:
                    self.left_motor.hold()
                    self.right_motor.hold()
                    self.ev3.speaker.beep()
                    self.state = "DONE"

            # FINISHED

            elif self.state == "DONE":
                self.left_motor.stop()
                self.right_motor.stop()

                # TODO: Test that all three objectives can
                # execute in one uninterrupted program run
                # within the 90-second time limit.


def main():
    lab1 = MyController()
    lab1.run()


if __name__ == "__main__":
    main()