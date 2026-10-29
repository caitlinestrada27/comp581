#!/usr/bin/env pybricks-micropython

"""Caitlin Estrada (730661469) and Madeline Clark (730650946)."""

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
        self.FORWARD_SPEED = 100
 
        # ---- Turning ----
        # With the gyro, turns are closed-loop: rotate until the gyro reads
        # 90 degrees (proportional slow-down, corrects overshoot, then holds).
        self.TURN_SPEED = 250          # max motor deg/s during a turn
        self.MIN_TURN_SPEED = 60       # min deg/s so the wheels never stall
        self.TURN_KP = 6.0             # motor deg/s per degree of heading error
        self.TURN_TOLERANCE = 1.0      # degrees
        # Encoder-only fallback (no gyro). Wheels scrub on a sharp turn, so the
        # robot under-rotates unless the wheel travel is scaled up. Raise this
        # if it still turns less than 90 degrees, lower it if it overshoots.
        self.TURN_SCALE = 1.15
        turn_arc = (math.pi / 2.0) * (self.WHEEL_BASE / 2.0)
        self.TURN_DEG_90 = self.TURN_SCALE * math.degrees(
            turn_arc / self.WHEEL_RADIUS)
 
        # Initial wall is measured by turning right, looking left, then
        # turning back. We drive by encoder distance since sensor faces left.
        # The robot was turning ~30cm from the wall with 0.20, so the sensor's
        # reading is ~10cm longer than the gap to the front of the robot.
        # 0.075 should give ~17cm at the turn. If the robot clips the wall
        # while turning, raise this a bit (0.01-0.02); if it turns too early,
        # lower it.
        self.TURN_STANDOFF = 0.075
        self.MIN_WALL_DIST = 0.40
        self.MAX_WALL_DIST = 1.60
        self.FALLBACK_WALL_DIST = 1.00
        self.drive_target = 0.0
        self.drive_start_left = 0
        self.drive_start_right = 0
 
        # Left wall desired distance and gentle heading-feedback controller.
        self.WALL_TARGET_DIST = 0.105
        self.DISTANCE_GAIN = 300.0   # deg heading commanded per meter error
        self.HEADING_GAIN = 3.0      # motor deg/s per deg heading error
        self.MAX_FOLLOW_TURN = 60.0  # max commanded heading (wall curving away)
        self.MAX_STEERING = 60.0     # max difference from base wheel speed
        self.HEADING_SAFETY_LIMIT = 100.0  # stop instead of spiraling
        # Slow down when far from the wall so the robot can turn tighter
        # (full slow-down at FAR_SLOW_ERR meters of error).
        self.FAR_SLOW_ERR = 0.15
        self.FAR_SLOW_FACTOR = 0.5   # speed multiplier when fully "far"
 
        # Wall-lost recovery: if the wall curves away out of sensor range,
        # keep moving and arc LEFT to find it again instead of stopping.
        self.MAX_VALID_DIST = 0.50   # readings beyond this = wall out of range
        self.LOST_ARC = 60.0         # motor deg/s added/removed to arc left
        self.LOST_TIMEOUT_MS = 4000  # give up searching after this long
        self.lost_timer = None
 
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
        """90-degree turn in place. Uses the gyro if available, else encoders."""
        if self.gyro is not None:
            self.rotate_gyro(right)
        else:
            self.rotate_encoder(right)
 
    def rotate_gyro(self, right, degrees=90.0):
        """Closed-loop turn: gyro angle is positive for a RIGHT turn."""
        sign = 1 if right else -1
        self.stop()
        wait(200)                      # let the robot be still before zeroing
        self.gyro.reset_angle(0)
        wait(100)
        timer = StopWatch()
        in_tol = 0
        while True:
            turned = sign * self.gyro.angle()
            error = degrees - turned   # >0 need more turn, <0 overshot
            if abs(error) <= self.TURN_TOLERANCE:
                in_tol += 1
                if in_tol >= 5:        # stay in tolerance for ~50 ms
                    break
                self.left_motor.stop()
                self.right_motor.stop()
            else:
                in_tol = 0
                speed = self.TURN_KP * error
                mag = max(self.MIN_TURN_SPEED, min(self.TURN_SPEED, abs(speed)))
                speed = mag if speed > 0 else -mag
                self.left_motor.run(sign * speed)
                self.right_motor.run(-sign * speed)
            if timer.time() > 8000:
                self.stop()
                raise RuntimeError("Turn timeout: check gyro/motor directions")
            wait(10)
        self.left_motor.hold()
        self.right_motor.hold()
        wait(300)
        self.stop()
 
    def rotate_encoder(self, right):
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
        self.left_motor.hold()
        self.right_motor.hold()
        wait(300)
        self.stop()
 
    def read_dist(self):
        """Median of five ultrasonic measurements, meters."""
        vals = []
        for _ in range(5):
            vals.append(self.ultrasonic.distance())
            wait(50)
        vals.sort()
        return vals[2] / 1000.0
 
    def reset_heading(self):
        if self.gyro is not None:
            self.gyro.reset_angle(0)
        self.heading_start_left = self.left_motor.angle()
        self.heading_start_right = self.right_motor.angle()
 
    def get_angle(self):
        """Positive heading means a RIGHT turn (verify gyro sign on robot)."""
        if self.gyro is not None:
            return self.gyro.angle()
        left_change = self.left_motor.angle() - self.heading_start_left
        right_change = self.right_motor.angle() - self.heading_start_right
        left_dist = math.radians(left_change) * self.WHEEL_RADIUS
        right_dist = math.radians(right_change) * self.WHEEL_RADIUS
        return math.degrees((left_dist - right_dist) / self.WHEEL_BASE)
 
    def show_once(self, label, dist):
        if self.last_screen_mode != label:
            self.ev3.screen.clear()
            self.ev3.screen.print(label)
            self.ev3.screen.print("{:.3f} m".format(dist))
            print(label, "{:.3f} m".format(dist))
            self.last_screen_mode = label
 
    def halt(self, reason):
        self.stop()
        self.ev3.screen.clear()
        self.ev3.screen.print(reason)
        print("STOP:", reason)
        self.ev3.speaker.beep(frequency=440, duration=400)
        self.state = "DONE"
 
    def run(self):
        while self.state != "DONE":
            wait(20)
 
            if self.state == "WAITING":
                self.stop()
                self.ev3.screen.clear()
                self.ev3.screen.print("Press CENTER")
                if Button.CENTER in self.ev3.buttons.pressed():
                    self.last_screen_mode = None
                    self.state = "SCAN"
 
            elif self.state == "SCAN":
                # Turn right so left-facing sensor points at front wall.
                self.rotate(True)
                wall = self.read_dist()
                self.last_screen_mode = None
                self.show_once("WALL AHEAD", wall)
                print("WALL DISTANCE AT START: {:.3f} m".format(wall))
                if not self.MIN_WALL_DIST <= wall <= self.MAX_WALL_DIST:
                    print("BAD READING: fallback {:.2f}m".format(
                        self.FALLBACK_WALL_DIST))
                    wall = self.FALLBACK_WALL_DIST
                self.rotate(False)
                self.drive_target = max(0.0, wall - self.TURN_STANDOFF)
                self.drive_start_left = self.left_motor.angle()
                self.drive_start_right = self.right_motor.angle()
                print("DRIVING {:.3f} m to standoff".format(self.drive_target))
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
                self.lost_timer = None
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
                if raw_dist <= 0.0 or raw_dist > self.MAX_VALID_DIST:
                    # Wall curved away: keep moving and arc LEFT to find it.
                    if self.lost_timer is None:
                        self.lost_timer = StopWatch()
                        self.filtered_dist = None
                    if self.lost_timer.time() > self.LOST_TIMEOUT_MS:
                        self.halt("WALL LOST")
                        continue
                    if abs(self.get_angle()) >= self.HEADING_SAFETY_LIMIT:
                        self.halt("HEADING LIMIT")
                        continue
                    self.left_motor.run(self.FORWARD_SPEED - self.LOST_ARC)
                    self.right_motor.run(self.FORWARD_SPEED + self.LOST_ARC)
                    self.show_once("SEARCH LEFT", raw_dist)
                    continue
                self.lost_timer = None
                self.bad_readings = 0
                if self.filtered_dist is None:
                    self.filtered_dist = raw_dist
                else:
                    # Less smoothing = less lag when the wall curves away.
                    self.filtered_dist = 0.5 * self.filtered_dist + 0.5 * raw_dist
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
                # Slow down when too far from the wall to tighten the turn.
                far = max(0.0, min(1.0, error / self.FAR_SLOW_ERR))
                base = self.FORWARD_SPEED * (
                    1.0 - (1.0 - self.FAR_SLOW_FACTOR) * far)
                self.left_motor.run(base + steering)
                self.right_motor.run(base - steering)
 
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
 
 
if __name__ == "__main__":
    main()
 