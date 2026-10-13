"""Caitlin Estrada (730661469) and Madeline Clark (730650946)."""

#!/usr/bin/env pybricks-micropython
import math 
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import wait

class MyController:
    # CALIBRATION CONSTANTS 
    # TODO @Maddie: Tune these constants for our robot.
    WHEEL_RADIUS = 0.027        # m  (measure your wheel)
    WHEEL_BASE = 0.0905         # m  (distance between wheel contact points)
    LEFT_DIR = Direction.CLOCKWISE        # flip to COUNTERCLOCKWISE if the
    RIGHT_DIR = Direction.CLOCKWISE       # robot drives backwards
 
    ODOMETRY_TARGET = 1.4       # m
    ODOM_BRAKE_COMP = 0.0       # m  stop this early to account for coasting
 
    RANGE_TARGET_MM = 400.0     # 40 cm from wall (nearest point of robot)
    SENSOR_OFFSET_MM = 0.0      # mm the ultrasonic sensor sits BEHIND the
                                # frontmost point of the robot (measure it)
    RANGE_BRAKE_COMP_MM = 0.0   # mm to stop early to account for coasting
    MAX_REVERSE = 0.6           # m  safety cutoff if the sensor misbehaves
 
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
        self.left_motor = Motor(Port.B, self.LEFT_DIRECTION)
        self.right_motor = Motor(Port.C, self.RIGHT_DIRECTION)
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

    # HELPER FUNCTIONS
    def update_button(self):
        """Call once per loop. button_press is True only on a NEW press."""
        now = Button.CENTER in self.ev3.buttons.pressed()
        self.button_press = now and not self.prev_button
        self.prev_button = now

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
 
    def halt_and_beep(self):
        self.halt()
        self.ev3.speaker.beep()
 
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

            # OBJECTIVE 1: ODOMETRY (1.4 METERS)
            if self.state == "WAITING_START":
                if self.button_press:
                    self.distance_traveled = 0.0
                    self.x, self.y, self.theta = 0.0, 0.0, 0.0
                    self.cmd_speed = 0.0
                    self.state = "DRIVE"


            elif self.state == "DRIVE":
                self.distance_traveled += d
                remaining = (self.ODOMETRY_TARGET - self.ODOM_BRAKE_COMP - self.distance_traveled)

                self.left_motor.run(self.cmd_speed)
                self.right_motor.run(self.cmd_speed)

                if remaining <= 0.0:
                    self.halt_and_beep()
                    self.state = "WAITING"
                else:
                    target = min(self.CRUISE_SPEED, self.MIN_SPEED + self.ODOM_DECEL_GAIN * remaining)
                    self.drive(self.ramp(target))

            # OBJECTIVE 2: RANGING (40 CM FROM WALL)

            elif self.state == "WAITING":
                if self.button_press:
                    self.reset_range_filter() # robot should be stationary here
                    self.cmd_speed = 0.0
                    self.state = "RANGING"

            elif self.state == "RANGING":
                front = self.read_front_mm()
                error = front - self.RANGE_TARGET_MM
 
                if error <= self.RANGE_BRAKE_COMP_MM:
                    self.halt_and_beep()
                    self.state = "WAITING_FOR_BUMP"
                else:
                    target = min(self.CRUISE_SPEED,
                                 self.MIN_SPEED + self.RANGE_GAIN * error)
                    self.drive(self.ramp(target))
 
            elif self.state == "WAITING_FOR_BUMP":
                if self.button_press:
                    self.cmd_speed = 0.0
                    self.state = "DRIVE_TO_WALL"

            # OBJECTIVE 3: BUMPING AND REVERSING

            elif self.state == "DRIVE_TO_WALL":
                if self.touch.pressed():
                    self.halt()
                    self.reset_range_filter()   # fresh readings at the wall
                    self.reverse_distance = 0.0
                    self.state = "REVERSE"
                else:
                    self.drive(self.ramp(self.BUMP_SPEED))
 
            elif self.state == "REVERSE":
                self.reverse_distance += abs(d)
                front = self.read_front_mm()
                error = self.RANGE_TARGET_MM - front   # mm still to back up
 
                if (error <= self.RANGE_BRAKE_COMP_MM
                        or self.reverse_distance >= self.MAX_REVERSE):
                    self.halt_and_beep()
                    self.state = "DONE"
                else:
                    speed = min(self.BUMP_SPEED + 40,
                                self.MIN_SPEED + self.RANGE_GAIN * error)
                    self.drive(self.ramp(-speed))
 
            # FINISHED
            elif self.state == "DONE":
                pass    # motors stay held in place

def main():
    lab1 = MyController()
    lab1.run()


if __name__ == "__main__":
    main()