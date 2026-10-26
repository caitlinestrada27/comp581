#!/usr/bin/env pybricks-micropython

import math

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.media.ev3dev import SoundFile, ImageFile

# Set to True when the gyro is plugged into S1.
USE_GYRO = True
 
 
                self.drive_start_left = self.left_motor.angle()
                self.drive_start_right = self.right_motor.angle()
                print("DRIVING {:.3f} m to 20cm standoff".format(self.drive_target))
                self.state = "DRIVE"

            elif self.state == "DRIVE":
                self.left_motor.run(self.APPROACH_SPEED + 1.5)
                self.right_motor.run(self.APPROACH_SPEED)
                dl = self.left_motor.angle() - self.drive_start_left
                dr = self.right_motor.angle() - self.drive_start_right
                travelled = math.radians((dl + dr) / 2.0) * self.WHEEL_RADIUS
                if travelled >= self.drive_target:
                    self.stop()
                    print("ARRIVED after {:.3f} m: turning RIGHT".format(travelled))
                    self.state = "TURN"

            elif self.state == "TURN":
                self.rotate(True)
                measured = self.read_dist()
                self.last_screen_mode = None
                self.show_once("AFTER TURN", measured)
                print("POST-TURN WALL DISTANCE: {:.3f} m (goal {:.3f})".format(
                    measured, self.WALL_TARGET_DIST))
                wait(1000)
                self.reset_heading()
                self.follow_dist = 0.0
                self.filtered_dist = None
                self.bad_readings = 0
                self.settle_counter = 0
                self.state = "SETTLE"

            elif self.state == "SETTLE":
                self.left_motor.run(self.FORWARD_SPEED)
                self.right_motor.run(self.FORWARD_SPEED)
                self.settle_counter += 1
                if self.settle_counter > 20:
                    # Don't count the settling movement twice.
                    self.prev_left = self.left_motor.angle()
                    self.prev_right = self.right_motor.angle()
                    self.ev3.screen.clear()
                    self.ev3.screen.print("FOLLOWING")
                    print("STATE: FOLLOW")
                    self.state = "FOLLOW"

            elif self.state == "FOLLOW":
                # Count linear wheel travel, not turn-in-place rotation.
                left_now = self.left_motor.angle()
                right_now = self.right_motor.angle()
                delta_left = left_now - self.prev_left
                delta_right = right_now - self.prev_right
                self.prev_left = left_now
                self.prev_right = right_now
                d_left = math.radians(delta_left) * self.WHEEL_RADIUS
                d_right = math.radians(delta_right) * self.WHEEL_RADIUS
                self.follow_dist += max(0.0, (d_left + d_right) / 2.0)
                if self.follow_dist >= self.FOLLOW_TARGET:
                    self.halt("COMPLETE")
                    break

                raw_dist = self.ultrasonic.distance() / 1000.0
                # Reject missing/unplausible wall readings; don't endlessly
                # turn left if the wall disappears from the sensor's view.
                if raw_dist <= 0.0 or raw_dist > 0.50:
                    self.bad_readings += 1
                    self.stop()
                    if self.bad_readings >= 10:
                        self.halt("WALL LOST")
                    continue
                self.bad_readings = 0
                if self.filtered_dist is None:
                    self.filtered_dist = raw_dist
                else:
                    self.filtered_dist = 0.7 * self.filtered_dist + 0.3 * raw_dist
                dist = self.filtered_dist

                angle = self.get_angle()
                if abs(angle) >= self.HEADING_SAFETY_LIMIT:
                    self.halt("HEADING LIMIT")
                    continue

                # Positive error = left wall too far away; turn LEFT
                # (negative desired heading). Negative error = turn RIGHT.
                error = dist - self.WALL_TARGET_DIST
                desired_angle = -self.DISTANCE_GAIN * error
                desired_angle = max(-self.MAX_FOLLOW_TURN,
                                    min(self.MAX_FOLLOW_TURN, desired_angle))
                steering = self.HEADING_GAIN * (desired_angle - angle)
                steering = max(-self.MAX_STEERING,
                               min(self.MAX_STEERING, steering))
                # Positive steering turns RIGHT; negative turns LEFT.
                self.left_motor.run(self.FORWARD_SPEED + steering)
                self.right_motor.run(self.FORWARD_SPEED - steering)

                if dist > 0.14:
                    label = "TOO_FAR"
                elif dist < 0.07:
                    label = "TOO_CLOSE"
                else:
                    label = "STRAIGHT"
                self.show_once(label, dist)
                self.last_report_counter += 1
                if self.last_report_counter >= 100:
                    print("FOLLOW dist={:.3f} heading={:.1f} target={:.1f} "
                          "travel={:.2f}".format(
                              dist, angle, desired_angle, self.follow_dist))
                    self.last_report_counter = 0
        self.stop()


def main():
    robot = None
    try:
        robot = MyController()
        robot.run()
    except Exception as e:
        if robot is not None:
            robot.stop()
        brick = EV3Brick()
        brick.screen.clear()
        msg = repr(e)
        brick.screen.print("ERROR:")
        for i in range(0, len(msg), 20):
            brick.screen.print(msg[i:i + 20])
        print("ERROR:", msg)
        wait(20000)
        raise
