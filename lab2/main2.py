#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class MyController:
    def __init__(self):
        self.ev3 = EV3Brick()
 
        self.left_motor = Motor(Port.C)
        self.right_motor = Motor(Port.B)
 
        self.ultrasonic = UltrasonicSensor(Port.S4)  # faces LEFT
        self.touch = TouchSensor(Port.S3)            # faces FRONT
        self.gyro = GyroSensor(Port.S1)              # faces UP
        wait(1000)
 
        self.follow_start_angle = 0.0
        self.MAX_FOLLOW_TURN = 45.0
 
        self.state = "WAITING"
 
        self.WHEEL_RADIUS = 0.03175
        self.WHEEL_BASE = 0.127
 
        self.APPROACH_SPEED = 200
        self.REVERSE_SPEED = 100
        self.TURN_SPEED = 200
        self.FORWARD_SPEED = 100
 
        self.REVERSE_STEPS = 95
 
        turn_arc = (math.pi / 2.0) * (self.WHEEL_BASE / 2.0)
        self.TURN_DEG_90 = 1.05 * math.degrees(turn_arc / self.WHEEL_RADIUS)
 
        self.turn_start_left = 0
        self.turn_start_right = 0
        self.reverse_counter = 0
        self.settle_counter = 0
        self.follow_dist = 0.0
        self.prev_left = 0
        self.prev_right = 0
 
        # Odometry distance at which we stop and beep.
        # Spring (2.0 m wall) used 2.5 here, which was tuned empirically.
        # Scaled by 2.2 / 2.0 = 1.1 -> 2.75. RETUNE on the real robot so the
        # stop lands within +/- 5 cm of 2.2 m along the wall.
        self.FOLLOW_TARGET = 2.75
 
        self.too_far_counter = 0
        self.TOO_FAR_TIMEOUT_LOOPS = 40
 
        self.TOO_CLOSE_THRESHOLD = 0.07  # triggers correction away from wall
        self.TOO_FAR_THRESHOLD = 0.14    # triggers correction toward wall
        self.STRAIGHT_LOW = 0.08         # hysteresis re-entry from TOO_CLOSE
        self.STRAIGHT_HIGH = 0.13        # hysteresis re-entry from TOO_FAR
 
        self.reference_dist = 0.0
        self.last_screen_mode = None
        self.wall_mode = "STRAIGHT"
 
        # straight-drive periodic print tracking
        self.straight_print_counter = 0
        self.STRAIGHT_PRINT_EVERY = 300
 
    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
 
    def show_once(self, label, dist):
        if self.last_screen_mode != label:
            self.ev3.screen.clear()
            self.ev3.screen.print(label)
            self.ev3.screen.print("{:.3f} m".format(dist))
            print(label, "{:.3f} m".format(dist))
            self.last_screen_mode = label
 
    def run(self):
        while True:
            wait(20)
            dist = self.ultrasonic.distance() / 1000.0
 
            if self.state == "WAITING":
                self.stop()
                self.ev3.screen.clear()
                self.ev3.screen.print("Press CENTER")
 
                if Button.CENTER in self.ev3.buttons.pressed():
                    self.reverse_counter = 0
                    self.wall_mode = "STRAIGHT"
                    self.last_screen_mode = None
                    self.state = "DRIVE"
 
            elif self.state == "DRIVE":
                self.left_motor.run(self.APPROACH_SPEED)
                self.right_motor.run(self.APPROACH_SPEED + 1.50)  # straight-line trim
 
                if self.touch.pressed():
                    self.stop()
                    self.reverse_counter = 0
                    self.state = "REVERSE"
 
            elif self.state == "REVERSE":
                self.left_motor.run(-self.REVERSE_SPEED)
                self.right_motor.run(-self.REVERSE_SPEED)
 
                self.reverse_counter += 1
 
                if self.reverse_counter >= self.REVERSE_STEPS:
                    self.stop()
                    self.turn_start_left = self.left_motor.angle()
                    self.turn_start_right = self.right_motor.angle()
                    # Reset once right before the turn starts.
                    self.gyro.reset_angle(0)
                    self.follow_start_angle = 0.0
                    self.state = "TURN"
 
            elif self.state == "TURN":
                left_change = abs(self.left_motor.angle() - self.turn_start_left)
                right_change = abs(self.right_motor.angle() - self.turn_start_right)
                turn_amount = (left_change + right_change) / 2.0
 
                if turn_amount < self.TURN_DEG_90:
                    self.left_motor.run(-self.TURN_SPEED)
                    self.right_motor.run(self.TURN_SPEED)
                else:
                    self.stop()
                    self.reference_dist = dist
                    self.last_screen_mode = None
                    self.show_once("REFERENCE", dist)
                    print("--- REFERENCE DISTANCE SET: {:.3f} m ---".format(self.reference_dist))
                    wait(1500)
 
                    self.wall_mode = "STRAIGHT"
                    self.last_screen_mode = None
 
                    self.prev_left = self.left_motor.angle()
                    self.prev_right = self.right_motor.angle()
                    self.follow_dist = 0.0
                    wait(300)
 
                    self.gyro.reset_angle(0)
 
                    self.state = "SETTLE"
                    self.settle_counter = 0
 
            elif self.state == "SETTLE":
                self.left_motor.run(self.FORWARD_SPEED)
                self.right_motor.run(self.FORWARD_SPEED)
 
                self.settle_counter += 1
                if self.settle_counter > 20:
                    self.state = "FOLLOW"
 
            elif self.state == "FOLLOW":
                current_angle = self.gyro.angle()
                if abs(current_angle) > self.MAX_FOLLOW_TURN:
                    print("GYRO LIMIT HIT: {:.1f} deg - forcing STRAIGHT".format(current_angle))
                    self.wall_mode = "STRAIGHT"
                    self.too_far_counter = 0
 
                # odometry: distance travelled while following
                delta_left = self.left_motor.angle() - self.prev_left
                delta_right = self.right_motor.angle() - self.prev_right
                self.prev_left = self.left_motor.angle()
                self.prev_right = self.right_motor.angle()
                d_left = math.radians(delta_left) * self.WHEEL_RADIUS
                d_right = math.radians(delta_right) * self.WHEEL_RADIUS
                self.follow_dist += abs((d_left + d_right) / 2.0)
 
                if self.follow_dist >= self.FOLLOW_TARGET:
                    self.stop()
                    self.ev3.speaker.beep(frequency=880, duration=400)
                    break
 
                # mode transitions (with hysteresis)
                if self.wall_mode == "STRAIGHT":
                    if dist < self.TOO_CLOSE_THRESHOLD:
                        print("TOO_CLOSE triggered: {:.3f} m (threshold: {:.3f} m)".format(
                            dist, self.TOO_CLOSE_THRESHOLD))
                        self.wall_mode = "TOO_CLOSE"
                        self.straight_print_counter = 0
 
                    elif dist > self.TOO_FAR_THRESHOLD:
                        print("TOO_FAR triggered: {:.3f} m (threshold: {:.3f} m)".format(
                            dist, self.TOO_FAR_THRESHOLD))
                        self.wall_mode = "TOO_FAR"
                        self.straight_print_counter = 0
 
                elif self.wall_mode == "TOO_CLOSE":
                    if dist >= self.STRAIGHT_LOW:
                        print("TOO_CLOSE resolved: {:.3f} m -> returning STRAIGHT".format(dist))
                        self.wall_mode = "STRAIGHT"
                        self.straight_print_counter = 0
 
                elif self.wall_mode == "TOO_FAR":
                    self.too_far_counter += 1
                    if self.too_far_counter >= self.TOO_FAR_TIMEOUT_LOOPS:
                        print("FORCING STRAIGHT (timeout)")
                        self.wall_mode = "STRAIGHT"
                        self.too_far_counter = 0
 
                    elif dist <= self.STRAIGHT_HIGH:
                        print("TOO_FAR resolved: {:.3f} m -> returning STRAIGHT".format(dist))
                        self.wall_mode = "STRAIGHT"
                        self.straight_print_counter = 0
 
                # motor commands
                if self.wall_mode == "STRAIGHT":
                    self.show_once("STRAIGHT", dist)
                    self.left_motor.run(self.FORWARD_SPEED)
                    self.right_motor.run(self.FORWARD_SPEED + 2)
 
                    self.straight_print_counter += 1
                    if self.straight_print_counter >= self.STRAIGHT_PRINT_EVERY:
                        self.straight_print_counter = 0
                        print("STRAIGHT: {:.3f} m (ref: {:.3f} m)".format(
                            dist, self.reference_dist))
 
                elif self.wall_mode == "TOO_CLOSE":
                    self.show_once("TOO_CLOSE", dist)
 
                    angle = self.gyro.angle()
                    error = self.reference_dist - dist
                    turn = 1500 * error
                    turn_scale = max(0.2, 1 - abs(angle) / self.MAX_FOLLOW_TURN)
                    turn = turn * turn_scale
                    turn = max(min(turn, 60), 10)
 
                    if abs(angle) > 30:
                        turn = turn * 0.8
 
                    base_speed = 80
                    self.left_motor.run(base_speed - turn)
                    self.right_motor.run(base_speed + turn)
 
                elif self.wall_mode == "TOO_FAR":
                    self.show_once("TOO_FAR", dist)
                    error = dist - self.reference_dist
 
                    if error < 0.01:
                        turn = 0
                    else:
                        turn = 1200 * error
 
                    angle = self.gyro.angle()
 
                    if abs(angle) > 30:
                        turn = turn * 0.5
 
                    turn_scale = max(0.2, 1 - abs(angle) / self.MAX_FOLLOW_TURN)
                    turn = turn * turn_scale
                    turn = max(min(turn, 60), 0)
 
                    base_speed = 80
                    self.left_motor.run(base_speed + turn)
                    self.right_motor.run(base_speed - turn)
 
 
def main():
    robot = MyController()
    robot.run()
 
 
if __name__ == "__main__":
    main()