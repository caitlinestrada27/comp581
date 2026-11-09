#!/usr/bin/env pybricks-micropython
# COMP 581 Lab 3 
# Caitlin Estrada (730661469) and Madeline Clark (730650946)

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch
import math


# =================== MAIN USER SETTINGS ================================
USE_GYRO = False  # True: use gyro S1; False: use ONLY wheel encoders
DEBUG = True      # Print position/distance information periodically

# Corner ARC SIZE: smaller radius -> a tighter turn around a convex corner.
# Adjust ONE parameter at a time; 0.09 m is a starting estimate.
ARC_RADIUS_M = 0.09
ARC_EASE_DEG = 70       # Maintain the tight arc until this heading change
ARC_EASE_FACTOR = 0.85 # After easing, keep 85% of the arc steering

# Hit-point loop detection (meters and degrees).
HIT_RADIUS_M = 0.24       # Must re-enter this circle around original hit
HIT_EXIT_RADIUS_M = 0.42  # Must first have LEFT this larger circle
MIN_LOOP_DIST_M = 1.40    # Must also have traveled at least this much
REQUIRE_HEADING_MATCH = True
HIT_HEADING_TOL_DEG = 50  # Compare with heading at beginning of first FOLLOW
# =======================================================================


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def wrap_degrees(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


class MyController:
    def __init__(self):
        self.ev3 = EV3Brick()
        self.left_motor = Motor(Port.C)
        self.right_motor = Motor(Port.D)
        self.touch_sensor = TouchSensor(Port.S3)
        self.ultrasonic = UltrasonicSensor(Port.S4)
        self.gyro = GyroSensor(Port.S1) if USE_GYRO else None
        self.clock = StopWatch()
        if USE_GYRO:
            wait(1000)  # Allow the stationary gyro to settle

        # Measured robot geometry; important for encoder-only heading.
        self.WHEEL_RADIUS = 0.03175  # meters
        self.WHEEL_BASE = 0.127     # distance between wheel tracks, meters

        self.APPROACH_SPEED = 200   # wheel degrees/second
        self.REVERSE_SPEED = 120
        self.TURN_SPEED = 200
        self.FOLLOW_SPEED = 140
        self.CORNER_SPEED = 120
        self.RETURN_SPEED = 180
        self.BACKUP_DIST = 0.10
        self.TRIM = 1.5           # small right-wheel speed correction
        self.KP_HEADING = 3.0

        # Straight-wall controller (left-hand wall).
        self.TARGET_DIST = 0.15
        self.KP_WALL = 250.0
        self.KP_YAW = 2.0
        self.MAX_STEER = 45.0
        self.LOST_DIST = 0.30
        self.MAX_US = 0.60

        # Convex-corner detection / reacquisition.
        self.LOST_CONFIRM = 0.02  # travel after losing wall, meters
        self.MIN_TRACK = 0.15
        self.SEE_DIST = 0.28
        self.SEE_LOOPS = 3
        self.MIN_ARC = 8.0
        self.MAX_ARC = 105.0
        self.KP_HUG = 400.0
        self.MAX_STEER_HUG = 90.0
        self.HUG_DIST = 0.20
        self.ARC_RADIUS_M = ARC_RADIUS_M
        self.ARC_EASE_DEG = ARC_EASE_DEG
        self.ARC_EASE_FACTOR = ARC_EASE_FACTOR

        # Completion thresholds.
        self.HIT_RADIUS = HIT_RADIUS_M
        self.HIT_EXIT_RADIUS = HIT_EXIT_RADIUS_M
        self.MIN_LOOP_DIST = MIN_LOOP_DIST_M
        self.HEADING_TOL = HIT_HEADING_TOL_DEG
        self.TIME_LIMIT_MS = 118000
        self.TURN_TIMEOUT_MS = 7000

        # Robot pose. x and y are meters. theta is CCW-positive radians.
        # heading_cw is a continuous (unwrapped) CW-positive angle in degrees.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.heading_cw = 0.0
        self.prev_left = self.left_motor.angle()
        self.prev_right = self.right_motor.angle()
        self.d_center = 0.0

        self.start_x = 0.0
        self.start_y = 0.0
        self.hit_x = 0.0
        self.hit_y = 0.0
        self.hit_saved = False
        self.left_hit_region = False
        self.first_follow_heading = None
        self.follow_dist = 0.0

        self.bump_x = 0.0
        self.bump_y = 0.0
        self.turn_target = 0.0
        self.turn_started = 0
        self.after_turn = "FOLLOW"
        self.state = "WAITING"

        # Filtered sonar and corner states.
        self.wall_f = 0.30
        self.wall_ref = 0.0
        self.track_dist = 0.0
        self.has_wall = False
        self.lost_phase = 0  # 0 normal, 2 arc, 3 hug, 4 search
        self.lost_count = 0.0
        self.see_count = 0
        self.hug_travel = 0.0
        self.lost_heading = 0.0
        self.debug_counter = 0
        self.waiting_displayed = False

    # -------------------------- Helpers --------------------------------

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def steer(self, speed, turn):
        # Positive turn curves RIGHT; negative turn curves LEFT.
        self.left_motor.run(speed + turn)
        self.right_motor.run(speed - turn + self.TRIM)

    def say(self, message):
        self.ev3.screen.clear()
        self.ev3.screen.print(message)
        print(message)

    def heading(self):
        return self.heading_cw

    def update_odometry(self):
        l = self.left_motor.angle()
        r = self.right_motor.angle()
        d_l = math.radians(l - self.prev_left) * self.WHEEL_RADIUS
        d_r = math.radians(r - self.prev_right) * self.WHEEL_RADIUS
        self.prev_left = l
        self.prev_right = r
        self.d_center = (d_l + d_r) / 2.0

        old_theta = self.theta
        if USE_GYRO:
            self.heading_cw = self.gyro.angle()  # CW positive for this mount
        else:
            # Right turn: left wheel advances relative to right wheel.
            delta_cw = (d_l - d_r) / self.WHEEL_BASE
            self.heading_cw += math.degrees(delta_cw)

        self.theta = -math.radians(self.heading_cw)
        middle_theta = (old_theta + self.theta) / 2.0
        self.x += self.d_center * math.cos(middle_theta)
        self.y += self.d_center * math.sin(middle_theta)

    def read_wall(self):
        raw = min(self.ultrasonic.distance() / 1000.0, self.MAX_US)
        self.wall_f = 0.5 * self.wall_f + 0.5 * raw
        return self.wall_f

    def dist_to(self, px, py):
        dx = self.x - px
        dy = self.y - py
        return math.sqrt(dx * dx + dy * dy)

    def begin_turn(self, signed_degrees, next_state):
        """Positive angle turns RIGHT; negative angle turns LEFT."""
        self.turn_target = self.heading() + signed_degrees
        self.turn_started = self.clock.time()
        self.after_turn = next_state
        self.state = "TURN"

    def face_start(self):
        bearing_ccw = math.atan2(self.start_y - self.y,
                                 self.start_x - self.x)
        desired_cw = -math.degrees(bearing_ccw)
        shortest_turn = wrap_degrees(desired_cw - self.heading())
        self.begin_turn(shortest_turn, "RETURN")

    def reset_follow_segment(self):
        self.wall_f = min(self.ultrasonic.distance() / 1000.0, self.MAX_US)
        self.wall_ref = self.heading()
        self.has_wall = False
        self.track_dist = 0.0
        self.lost_phase = 0
        self.lost_count = 0.0
        self.see_count = 0
        if self.first_follow_heading is None:
            # Record only ONCE, after the first right turn at the obstacle.
            self.first_follow_heading = self.heading()

    # ----------------------- Wall following -----------------------------

    def wall_follow(self, wall):
        # Wall is on robot's LEFT. A leftward yaw makes CW heading smaller.
        yaw_left = clamp(math.radians(self.wall_ref - self.heading()),
                         -0.7, 0.7)
        approx_perpendicular_dist = wall * math.cos(yaw_left)
        error = self.TARGET_DIST - approx_perpendicular_dist
        turn = self.KP_WALL * error + self.KP_YAW * math.degrees(yaw_left)
        self.steer(self.FOLLOW_SPEED, clamp(turn, -self.MAX_STEER,
                                           self.MAX_STEER))
        if abs(error) < 0.03:
            self.wall_ref += 0.03 * (self.heading() - self.wall_ref)

    def start_arc(self):
        self.say("ARC")
        self.lost_phase = 2
        self.lost_count = 0.0
        self.see_count = 0
        self.lost_heading = self.heading()

    def start_hug(self):
        self.say("HUG")
        self.lost_phase = 3
        self.hug_travel = 0.0
        self.lost_count = 0.0
        self.see_count = 0

    def follow_step(self, wall):
        # Phase 0: normal wall following.
        if self.lost_phase == 0:
            if wall <= self.LOST_DIST:
                self.track_dist += abs(self.d_center)
                if self.track_dist >= self.MIN_TRACK:
                    self.has_wall = True
                self.lost_count = 0.0
                self.wall_follow(wall)
            elif not self.has_wall:
                self.steer(self.FOLLOW_SPEED * 0.7, 0)
            else:
                self.lost_count += abs(self.d_center)
                if self.lost_count < self.LOST_CONFIRM:
                    self.steer(self.FOLLOW_SPEED, 0)
                else:
                    self.start_arc()

        # Phase 2: arc LEFT around a convex corner to reacquire the wall.
        elif self.lost_phase == 2:
            turned_left = self.lost_heading - self.heading()
            if wall < self.SEE_DIST and turned_left >= self.MIN_ARC:
                self.see_count += 1
            else:
                self.see_count = 0

            if self.see_count >= self.SEE_LOOPS:
                self.start_hug()
            elif turned_left >= self.MAX_ARC:
                self.say("SEARCH")
                self.lost_phase = 4
            else:
                # Approximate turning radius for differential drive:
                # radius = (wheelbase/2) * speed/abs(steer).
                steer_magnitude = (self.CORNER_SPEED * self.WHEEL_BASE /
                                   (2.0 * self.ARC_RADIUS_M))
                if turned_left >= self.ARC_EASE_DEG:
                    steer_magnitude *= self.ARC_EASE_FACTOR
                steer_magnitude = clamp(steer_magnitude, 5.0,
                                        self.CORNER_SPEED * 0.95)
                self.steer(self.CORNER_SPEED, -steer_magnitude)

        # Phase 3: temporarily hug the new wall before regular tracking.
        elif self.lost_phase == 3:
            self.wall_ref = self.heading()
            self.hug_travel += abs(self.d_center)
            if wall > self.LOST_DIST:
                self.lost_count += abs(self.d_center)
                if self.lost_count >= self.LOST_CONFIRM:
                    self.start_arc()
                else:
                    self.steer(self.CORNER_SPEED, 0)
            else:
                self.lost_count = 0.0
                error = self.TARGET_DIST - wall
                turn = clamp(self.KP_HUG * error,
                             -self.MAX_STEER_HUG, self.MAX_STEER_HUG)
                self.steer(self.CORNER_SPEED, turn)
                if self.hug_travel >= self.HUG_DIST:
                    self.lost_phase = 0
                    self.lost_count = 0.0

        # Phase 4: no wall after the maximum arc; search forward.
        else:
            if wall <= self.LOST_DIST:
                self.start_hug()
            else:
                self.steer(self.CORNER_SPEED, -8)

    # --------------------- Loop completion ------------------------------

    def back_at_hit(self):
        if not self.hit_saved:
            return False
        distance = self.dist_to(self.hit_x, self.hit_y)

        # Prevent treating the first pass near the hit as a completed loop.
        if distance >= self.HIT_EXIT_RADIUS:
            self.left_hit_region = True

        if not self.left_hit_region or self.follow_dist < self.MIN_LOOP_DIST:
            return False
        if distance > self.HIT_RADIUS:
            return False
        if REQUIRE_HEADING_MATCH and self.first_follow_heading is not None:
            diff = wrap_degrees(self.heading() - self.first_follow_heading)
            if abs(diff) > self.HEADING_TOL:
                return False
        return True

    # ------------------------- Main loop --------------------------------

    def run(self):
        while True:
            wait(20)

            if self.state == "WAITING":
                if not self.waiting_displayed:
                    self.stop()
                    self.say("Press CENTER")
                    self.waiting_displayed = True
                if Button.CENTER in self.ev3.buttons.pressed():
                    wait(500)
                    if USE_GYRO:
                        self.gyro.reset_angle(0)
                    self.x = 0.0
                    self.y = 0.0
                    self.theta = 0.0
                    self.heading_cw = 0.0
                    self.prev_left = self.left_motor.angle()
                    self.prev_right = self.right_motor.angle()
                    self.follow_dist = 0.0
                    self.hit_saved = False
                    self.left_hit_region = False
                    self.first_follow_heading = None
                    self.debug_counter = 0
                    self.clock.reset()
                    self.state = "DRIVE"
                continue

            if self.state == "DONE":
                self.stop()
                break

            if self.clock.time() > self.TIME_LIMIT_MS:
                self.stop()
                self.say("TIME UP")
                break

            self.update_odometry()
            wall = self.read_wall()

            if self.state == "DRIVE":
                # Maintain initial 0-degree heading while approaching.
                turn = clamp(-self.KP_HEADING * self.heading(), -40, 40)
                self.steer(self.APPROACH_SPEED, turn)
                if self.touch_sensor.pressed():
                    self.stop()
                    self.ev3.speaker.beep()
                    self.hit_x, self.hit_y = self.x, self.y
                    self.hit_saved = True
                    self.bump_x, self.bump_y = self.x, self.y
                    self.say("HIT")
                    self.state = "BACKUP"

            elif self.state == "BACKUP":
                self.left_motor.run(-self.REVERSE_SPEED)
                self.right_motor.run(-self.REVERSE_SPEED)
                if self.dist_to(self.bump_x, self.bump_y) >= self.BACKUP_DIST:
                    self.stop()
                    self.begin_turn(90.0, "FOLLOW")

            elif self.state == "TURN":
                remaining = self.turn_target - self.heading()
                if abs(remaining) > 2.0:
                    if self.clock.time() - self.turn_started > self.TURN_TIMEOUT_MS:
                        self.stop()
                        self.say("TURN TIMEOUT")
                        self.state = "DONE"
                        continue
                    speed = clamp(abs(remaining) * 5.0, 40.0, self.TURN_SPEED)
                    if remaining > 0:
                        self.left_motor.run(speed)  # clockwise/right
                        self.right_motor.run(-speed)
                    else:
                        self.left_motor.run(-speed)  # counterclockwise/left
                        self.right_motor.run(speed)
                else:
                    self.stop()
                    wait(100)
                    if self.after_turn == "FOLLOW":
                        self.reset_follow_segment()
                        self.say("FOLLOW")
                    else:
                        self.say("RETURN")
                    self.state = self.after_turn

            elif self.state == "FOLLOW":
                self.follow_dist += abs(self.d_center)
                if self.touch_sensor.pressed():
                    self.stop()
                    self.bump_x, self.bump_y = self.x, self.y
                    self.state = "BACKUP"
                    continue

                self.follow_step(wall)
                self.debug_counter += 1
                if DEBUG and self.debug_counter % 50 == 0:
                    print("FOLLOW distance={:.2f}m hit={:.2f}m "
                          "heading={:.0f}deg arc_phase={}".format(
                              self.follow_dist,
                              self.dist_to(self.hit_x, self.hit_y),
                              self.heading(), self.lost_phase))

                if self.back_at_hit():
                    self.stop()
                    self.ev3.speaker.beep()
                    self.say("BACK AT HIT")
                    wait(300)
                    self.face_start()

            elif self.state == "RETURN":
                dx = self.start_x - self.x
                dy = self.start_y - self.y
                distance = math.sqrt(dx * dx + dy * dy)
                if distance <= 0.05:
                    self.stop()
                    self.ev3.speaker.beep()
                    self.say("AT START")
                    self.state = "DONE"
                    continue

                desired_ccw = math.atan2(dy, dx)
                heading_error = wrap_pi(desired_ccw - self.theta)
                turn = clamp(-3.0 * math.degrees(heading_error), -60, 60)
                speed = clamp(distance * 500, 60, self.RETURN_SPEED)
                self.steer(speed, turn)

        self.stop()


def main():
    robot = MyController()
    try:
        robot.run()
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
