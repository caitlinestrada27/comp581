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
        self.TURN_SPEED = 200
        self.FORWARD_SPEED = 100

        turn_arc = (math.pi / 2.0) * (self.WHEEL_BASE / 2.0)
        self.TURN_DEG_90 = 1.05 * math.degrees(turn_arc / self.WHEEL_RADIUS)

        # Initial wall is measured by turning right, looking left, then
        # turning back. We drive by encoder distance since sensor faces left.
        # Previous commanded 0.20m stopped ~0.30m physically.
        # Compensate by driving another ~0.10m; verify measured clearance.
        self.TURN_STANDOFF = 0.10  # empirical correction: 20cm command stopped ~30cm
        self.MIN_WALL_DIST = 0.40
        self.MAX_WALL_DIST = 1.60
        self.FALLBACK_WALL_DIST = 1.00
        self.drive_target = 0.0
        self.drive_start_left = 0
        self.drive_start_right = 0

        # Left wall desired distance and gentle heading-feedback controller.
        self.WALL_TARGET_DIST = 0.105
        self.DISTANCE_GAIN = 180.0   # deg heading commanded per meter error
        self.HEADING_GAIN = 4.0      # motor deg/s per deg heading error
        self.MAX_FOLLOW_TURN = 22.0  # max commanded heading from wall direction
        self.MAX_STEERING = 35.0     # max difference from base wheel speed
        self.HEADING_SAFETY_LIMIT = 60.0  # stop instead of spiraling

        # Target actual forward progress along the original wall direction,
        # not the older empirically tuned 2.75m wheel path.
        # Verify against markings on the floor; wheel slip creates errors.
        self.FOLLOW_TARGET = 2.20
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
        self.waiting_screen_shown = False
        print("INITIALIZED - waiting for EV3 CENTER button")

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
        self.ev3.screen.print("{:.2f} m".format(self.follow_dist))
        print("STOP:", reason, "distance:", self.follow_dist)
        self.ev3.speaker.beep(frequency=440, duration=400)
        self.state = "DONE"

    def run(self):
        while self.state != "DONE":
            wait(20)

            if self.state == "WAITING":
                if not self.waiting_screen_shown:
                    self.stop()
                    self.ev3.screen.clear()
                    self.ev3.screen.print("READY")
                    self.ev3.screen.print("Press CENTER")
                    self.waiting_screen_shown = True
                    print("READY: press center button on EV3 brick")
                buttons = self.ev3.buttons.pressed()
                if Button.CENTER in buttons:
                    print("CENTER DETECTED - starting scan")
                    self.ev3.screen.clear()
                    self.ev3.screen.print("STARTING")
                    self.last_screen_mode = None
                    self.state = "SCAN"
                    wait(250)  # debounce, and make STARTING readable

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
                print("DRIVING {:.3f} m (approach calibration 10cm)".format(self.drive_target))
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
                # Project movement onto the original wall direction.
                # This excludes extra distance from steering/corrections.
                forward_motion = (d_left + d_right) / 2.0
                heading_rad = math.radians(self.get_angle())
                self.follow_dist += max(0.0, forward_motion * math.cos(heading_rad))
                if self.follow_dist >= self.FOLLOW_TARGET:
                    self.halt("COMPLETE")
                    break

                raw_dist = self.ultrasonic.distance() / 1000.0
                angle = self.get_angle()
                if abs(angle) >= self.HEADING_SAFETY_LIMIT:
                    self.halt("HEADING LIMIT")
                    continue

                # Missing ultrasonic returns are common near openings/corners.
                # Do NOT end a run before 2.2m just because the wall is lost.
                # While missing, return to the original parallel heading.
                valid_wall = 0.02 <= raw_dist <= 0.50
                if valid_wall:
                    self.bad_readings = 0
                    if self.filtered_dist is None:
                        self.filtered_dist = raw_dist
                    else:
                        # React faster to a wall that moves away.
                        self.filtered_dist = (0.35 * self.filtered_dist
                                              + 0.65 * raw_dist)
                    dist = self.filtered_dist
                    error = dist - self.WALL_TARGET_DIST
                    desired_angle = -self.DISTANCE_GAIN * error
                    desired_angle = max(-self.MAX_FOLLOW_TURN,
                                        min(self.MAX_FOLLOW_TURN, desired_angle))
                else:
                    self.bad_readings += 1
                    dist = raw_dist
                    self.filtered_dist = None
                    desired_angle = 0.0
                    if self.bad_readings == 1:
                        print("WALL NOT SEEN: continuing until 2.20m")

                steering = self.HEADING_GAIN * (desired_angle - angle)
                steering = max(-self.MAX_STEERING,
                               min(self.MAX_STEERING, steering))
                self.left_motor.run(self.FORWARD_SPEED + steering)
                self.right_motor.run(self.FORWARD_SPEED - steering)

                if not valid_wall:
                    label = "WALL LOST"
                elif dist > 0.14:
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
        msg = repr(e)
        print("ERROR:", msg)
        try:
            brick = robot.ev3 if robot is not None else EV3Brick()
            brick.screen.clear()
            brick.screen.print("ERROR:")
            for i in range(0, len(msg), 20):
                brick.screen.print(msg[i:i + 20])
        except Exception as display_error:
            print("Could not show error on brick:", repr(display_error))
        wait(20000)
        raise


if __name__ == "__main__":
    main()