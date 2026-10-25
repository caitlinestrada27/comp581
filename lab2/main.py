#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

class MyController:
    # CALIBRATION CONSTANTS 
    # TODO @Maddie: Tune these constants for our robot.
    WHEEL_RADIUS = 0.028        # m  (measure your wheel)
    WHEEL_BASE = 0.118         # m  (distance between wheel contact points)
    LEFT_DIR = Direction.CLOCKWISE        # flip to COUNTERCLOCKWISE if the
    RIGHT_DIR = Direction.CLOCKWISE       # robot drives backwards
 
 
    RANGE_TARGET_MM = 220.0     # 40 cm from wall (nearest point of robot)
    SENSOR_OFFSET_MM = 0.0      # mm the ultrasonic sensor sits BEHIND the
                                # frontmost point of the robot (measure it)
    RANGE_BRAKE_COMP_MM = 0.0   # mm to stop early to account for coasting
    WALL_FOLLOW_DISTANCE = 220.0   # mm distance to follow alongside the wall once turned
 
    CRUISE_SPEED = 360          # deg/s  top speed (~0.17 m/s with 27 mm wheel)
    MIN_SPEED = 90              # deg/s  creep speed near targets
    BUMP_SPEED = 180            # deg/s  approach speed for the bump
    ACCEL = 8                   # deg/s added per loop when speeding up
    ODOM_DECEL_GAIN = 1500.0    # deg/s per m of remaining distance
    RANGE_GAIN = 0.5            # deg/s per mm of remaining range error
 
    HEADING_KP = 400.0          # deg/s per rad of heading error
    MAX_CORR = 40.0             # deg/s cap on heading correction
 
    FILTER_SIZE = 5             # median filter window for the ultrasonic
    
    def __init__(self):
        self.ev3 = EV3Brick()
        # TODO @Maddie: Make sure assembled robot matches the Port assignemnts
        self.left_motor = Motor(Port.B, self.LEFT_DIR)
        self.right_motor = Motor(Port.C, self.RIGHT_DIR)
        self.ultrasonic = UltrasonicSensor(Port.S4)
        self.touch = TouchSensor(Port.S3)

        self.state = "WAITING_START"
        self.distance_traveled = 0.0
        self.reverse_distance = 0.0
        self.prev_left = None
        self.prev_right = None
        self.x, self.y, self.theta = 0.0, 0.0, 0.0

        self.cmd_speed = 0.0
        self.range_buf = []

        # Button edge tracking. Start with the CURRENT button state so a 
        # center press used ot launch the program doesn't count as a start.
        self.prev_button = Button.CENTER in self.ev3.buttons.pressed()
        self.button_press = False

    def compute_position(self, delta_left, delta_right):
        d_left = math.radians(delta_left) * self.WHEEL_RADIUS
        d_right = math.radians(delta_right) * self.WHEEL_RADIUS
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.WHEEL_BASE
 
        # Midpoint heading for better differential-drive odometry.
        mid_theta = self.theta + d_theta / 2.0
        self.x += d_center * math.cos(mid_theta)
        self.y += d_center * math.sin(mid_theta)
        self.theta += d_theta
        return d_center
    

    def ramp(self, target):
        """Limit acceleration (avoids wheel slip); slowing is immediate."""
        if abs(target) <= abs(self.cmd_speed):
            self.cmd_speed = target
        elif target > self.cmd_speed:
            self.cmd_speed = min(target, self.cmd_speed + self.ACCEL)
        else:
            self.cmd_speed = max(target, self.cmd_speed - self.ACCEL)
        return self.cmd_speed
 
    def drive(self, speed):
        """Drive straight; use estimated heading to trim the two wheels.
        The same formula works forward and in reverse."""
        scale = min(1.0, abs(speed) / self.CRUISE_SPEED)
        corr = self.HEADING_KP * self.theta * scale
        corr = max(-self.MAX_CORR, min(self.MAX_CORR, corr))
        self.left_motor.run(speed + corr)
        self.right_motor.run(speed - corr)

    def halt(self):
        self.left_motor.hold()
        self.right_motor.hold()
        self.cmd_speed = 0.0

    def reset_range_filter(self):
        """Fill the median filter with fresh readings (robot is stopped)."""
        self.range_buf = []
        for _ in range(self.FILTER_SIZE):
            wait(40)
            self.range_buf.append(self.ultrasonic.distance())
 
    def read_front_mm(self):
        """Median-filtered distance (mm) from the frontmost point of the
        robot to the wall."""
        self.range_buf.pop(0)
        self.range_buf.append(self.ultrasonic.distance())
        median = sorted(self.range_buf)[len(self.range_buf) // 2]
        return median - self.SENSOR_OFFSET_MM
    
    # MAIN LOOP
    def run(self):
        while True:
            wait(10)
            self.update_button()

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
            


