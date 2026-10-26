#!/usr/bin/env pybricks-micropython

import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import wait, StopWatch

USE_GYRO = True


class MyController:
    def __init__(self):
        self.ev3 = EV3Brick()
        self.left_motor = Motor(Port.B, Direction.CLOCKWISE)
        self.right_motor = Motor(Port.C, Direction.CLOCKWISE)
        self.ultrasonic = UltrasonicSensor(Port.S4)  # Fixed facing LEFT
        self.gyro = GyroSensor(Port.S1) if USE_GYRO else None
        wait(1000)

        # Verify these two physical dimensions on the actual robot.
        self.WHEEL_RADIUS = 0.03175     # meters
        self.WHEEL_BASE = 0.127        # meters between wheel contact tracks
        self.APPROACH_SPEED = 200      # motor degrees / second
        self.TURN_SPEED = 200
        self.FORWARD_SPEED = 100

        turn_arc = (math.pi / 2.0) * (self.WHEEL_BASE / 2.0)
        self.TURN_DEG_90 = 1.05 * math.degrees(turn_arc / self.WHEEL_RADIUS)

        # Initial wall is measured by turning right, looking left, then
        # turning back. We drive by encoder distance since sensor faces left.
        # 0.20m gives clearance before the final right turn (< 0.30m).
        self.TURN_STANDOFF = 0.20
        self.MIN_WALL_DIST = 0.40
        self.MAX_WALL_DIST = 1.60
        self.FALLBACK_WALL_DIST = 1.00
        self.drive_target = 0.0
        self.drive_start_left = 0
        self.drive_start_right = 0

        # Left wall desired distance and gentle heading-feedback controller.
        self.WALL_TARGET_DIST = 0.105
        self.DISTANCE_GAIN = 100.0   # deg heading commanded per meter error
        self.HEADING_GAIN = 3.0      # motor deg/s per deg heading error
        self.MAX_FOLLOW_TURN = 15.0  # max commanded heading from wall direction
        self.MAX_STEERING = 20.0     # max difference from base wheel speed
        self.HEADING_SAFETY_LIMIT = 50.0  # stop instead of spiraling

        # Original stopping distance retained for your course setup.
        # Calibrate this against 2.2 m of actual distance along the wall.
        self.FOLLOW_TARGET = 2.75
        self.follow_dist = 0.0
        self.prev_left = 0
        self.prev_right = 0
        self.heading_start_left = 0
        self.heading_start_right = 0

        self.state = "WAITING"
        self.settle_counter = 0
        self.last_screen_mode = None
        self.last_report_counter = 0
        self.filtered_dist = None
        self.bad_readings = 0

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def rotate(self, right):
        """Approximate 90-degree turn in place using motor encoders."""
        start_left = self.left_motor.angle()
        start_right = self.right_motor.angle()
        sign = 1 if right else -1
        timer = StopWatch()
        while True:
            left_change = abs(self.left_motor.angle() - start_left)
            right_change = abs(self.right_motor.angle() - start_right)
            if (left_change + right_change) / 2.0 >= self.TURN_DEG_90:
                break
            if timer.time() > 8000:
                self.stop()
                raise RuntimeError("Turn timeout: check wheels/motor directions")
            self.left_motor.run(sign * self.TURN_SPEED)
            self.right_motor.run(-sign * self.TURN_SPEED)
            wait(10)
        self.stop()
        wait(300)

    def read_dist(self):
        """Median of five ultrasonic measurements, meters."""