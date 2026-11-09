#!/usr/bin/env pybricks-micropython
# COMP 581 Lab 3 
# Caitlin Estrada (730661469) and Madeline Clark (730650946)

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch
import math


# ====================== MEASURE THESE ON YOUR ROBOT ======================
WHEEL_RADIUS_M = 0.03175    # wheel radius
WHEEL_BASE_M = 0.127        # distance between the two wheel contact patches
SONAR_AHEAD_M = 0.09        # how far the sonar is IN FRONT of the wheel axle
MEASURE_AHEAD_M = 0.12      # axle to measuring point (frontmost surface)
ARC_RADIUS_M = 0.17         # TARGET_DIST_M + (robot centerline to sonar face)
BUMP_ARC_RADIUS_M = 0.11    # tighter arc when no wall is seen after a bump turn

# ============================ WALL FOLLOWING =============================
TARGET_DIST_M = 0.12        # desired sonar reading while following
LOST_DIST_M = 0.30          # reading above this = wall not visible
TOO_CLOSE_M = 0.07          # below this always steer away
KP_WALL = 600.0             # wheel deg/s of steer per meter of error
KD_WALL = 500.0             # wheel deg/s of steer per (m/s) of error change
FOLLOW_SPEED = 220.0        # wheel deg/s (about 12 cm/s)
ARC_SPEED = 180.0           # wheel deg/s while rounding an outside corner
LOST_CONFIRM_M = 0.02       # travel with no wall before reacting
MIN_TRACK_M = 0.10          # wall must be tracked this far to use ADVANCE
ARC_MAX_TURN_DEG = 330.0    # give up if a full circle finds no wall

# ============================ OTHER MOTIONS ==============================
APPROACH_SPEED = 220.0
REVERSE_SPEED = 150.0
TURN_SPEED = 200.0
RETURN_SPEED = 220.0
BACKUP_DIST_M = 0.08
BUMP_TURN_DEG = 90.0        # right turn after every bumper hit
KP_HEADING = 3.0
TRIM = 1.5                  # small right-wheel speed correction

# =========================== LOOP COMPLETION =============================
MIN_LOOP_DIST_M = 1.20      # minimum wall-following travel
LOOP_MIN_TURN_DEG = 250.0   # net LEFT turn required before checking hit point
LOOP_FORCE_TURN_DEG = 450.0 # odometry drifted badly: call the loop done anyway
HIT_RADIUS_M = 0.35         # closest approach must be inside this radius

TIME_LIMIT_MS = 118000
LOOP_MS = 20
DEBUG = True
# =========================================================================


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_degrees(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


class Robot:
    def __init__(self):
        self.ev3 = EV3Brick()
        self.left_motor = Motor(Port.C)
        self.right_motor = Motor(Port.D)
        self.touch = TouchSensor(Port.S3)
        self.sonar = UltrasonicSensor(Port.S4)
        self.clock = StopWatch()

        # Pose from wheel encoders. heading_cw is continuous (not wrapped),
        # in degrees, positive when the robot has turned RIGHT.
        self.x = 0.0
        self.y = 0.0
        self.heading_cw = 0.0
        self.d_center = 0.0
        self.prev_left = 0
        self.prev_right = 0

        self.state = "DRIVE"
        self.hit_x = 0.0
        self.hit_y = 0.0
        self.bump_x = 0.0
        self.bump_y = 0.0
        self.state_start_ms = 0

        self.turn_target = 0.0
        self.after_turn = "FOLLOW"

        # Wall following
        self.mode = "TRACK"
        self.readings = [LOST_DIST_M, LOST_DIST_M, LOST_DIST_M]
        self.prev_wall = None
        self.d_filtered = 0.0
        self.track_dist = 0.0
        self.lost_travel = 0.0
        self.advance_dist = 0.0
        self.arc_start_heading = 0.0
        self.arc_radius = ARC_RADIUS_M
        self.follow_dist = 0.0
        self.first_follow_heading = None
        self.loop_min_d = 99.0
        self.stuck_start_ms = 0
        self.stuck_travel = 0.0

        self.return_min_d = 99.0
        self.debug_counter = 0

    # ------------------------------ helpers ------------------------------

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def steer(self, speed, turn):
        # Positive turn curves RIGHT; negative turn curves LEFT.
        self.left_motor.run(speed + turn)
        self.right_motor.run(speed - turn + TRIM)

    def say(self, message):
        self.ev3.screen.clear()
        self.ev3.screen.print(message)
        print(message)

    def update_odometry(self):
        l = self.left_motor.angle()
        r = self.right_motor.angle()
        d_l = math.radians(l - self.prev_left) * WHEEL_RADIUS_M
        d_r = math.radians(r - self.prev_right) * WHEEL_RADIUS_M
        self.prev_left = l
        self.prev_right = r
        self.d_center = (d_l + d_r) / 2.0
        d_heading = math.degrees((d_l - d_r) / WHEEL_BASE_M)
        mid = -math.radians(self.heading_cw + d_heading / 2.0)
        self.heading_cw += d_heading
        self.x += self.d_center * math.cos(mid)
        self.y += self.d_center * math.sin(mid)

    def read_wall(self):
        """Median of the last three sonar readings, in meters."""
        self.readings.pop(0)
        self.readings.append(self.sonar.distance() / 1000.0)
        return sorted(self.readings)[1]

    def dist_to(self, px, py):
        dx = self.x - px
        dy = self.y - py
        return math.sqrt(dx * dx + dy * dy)

    def bearing_error_to(self, px, py):
        """Degrees the robot must turn (right positive) to face (px, py)."""
        desired_cw = -math.degrees(math.atan2(py - self.y, px - self.x))
        return wrap_degrees(desired_cw - self.heading_cw)

    def set_state(self, state):
        self.state = state
        self.state_start_ms = self.clock.time()

    def begin_backup(self):
        self.stop()
        self.bump_x, self.bump_y = self.x, self.y
        self.set_state("BACKUP")

    def begin_turn(self, signed_degrees, next_state):
        """Positive turns RIGHT, negative turns LEFT."""
        self.turn_target = self.heading_cw + signed_degrees
        self.after_turn = next_state
        self.set_state("TURN")

    def begin_follow(self):
        if self.first_follow_heading is None:
            self.first_follow_heading = self.heading_cw
        self.mode = "TRACK"
        self.track_dist = 0.0
        self.lost_travel = 0.0
        self.prev_wall = None
        self.d_filtered = 0.0
        for _ in range(3):
            self.read_wall()
            wait(15)
        self.set_state("FOLLOW")
        self.stuck_start_ms = self.clock.time()
        self.stuck_travel = 0.0
        self.say("FOLLOW")

    def begin_peel(self):
        self.set_state("PEEL")
        self.say("PEEL")

    # --------------------------- wall following --------------------------

    def start_arc(self, radius):
        self.mode = "ARC"
        self.arc_radius = radius
        self.arc_start_heading = self.heading_cw
        if DEBUG:
            print("ARC")

    def track(self, wall, dt):
        """PD control on the sonar distance. Wall is on the LEFT."""
        error = wall - TARGET_DIST_M              # + = too far from the wall
        if self.prev_wall is not None:
            raw_d = (wall - self.prev_wall) / dt
            self.d_filtered = 0.7 * self.d_filtered + 0.3 * raw_d
        self.prev_wall = wall
        d_term = clamp(KD_WALL * self.d_filtered, -40.0, 40.0)
        turn = -(KP_WALL * error + d_term)        # too far -> steer LEFT (-)
        # Never curve left tighter than the corner arc; allow a harder right.
        left_limit = FOLLOW_SPEED * WHEEL_BASE_M / (2.0 * ARC_RADIUS_M)
        turn = clamp(turn, -left_limit, 0.5 * FOLLOW_SPEED)
        if wall < TOO_CLOSE_M:
            turn = max(turn, 0.3 * FOLLOW_SPEED)
        self.steer(FOLLOW_SPEED, turn)

    def follow_step(self, wall, dt):
        seen = wall <= LOST_DIST_M
        step = abs(self.d_center)

        if self.mode == "TRACK":
            if seen:
                self.lost_travel = 0.0
                self.track_dist += step
                self.track(wall, dt)
            else:
                self.lost_travel += step
                self.prev_wall = None
                self.d_filtered = 0.0
                if self.lost_travel < LOST_CONFIRM_M:
                    self.steer(ARC_SPEED, 0.0)
                elif self.track_dist >= MIN_TRACK_M:
                    # Sonar just passed an outside corner. Drive straight
                    # until the axle reaches the corner, then arc around it.
                    self.mode = "ADVANCE"
                    self.advance_dist = 0.0
                    if DEBUG:
                        print("ADVANCE")
                else:
                    # No wall right after a bump turn: we are farther out
                    # than usual, so curl back toward the wall more tightly.
                    self.start_arc(BUMP_ARC_RADIUS_M)

        elif self.mode == "ADVANCE":
            self.advance_dist += step
            if seen:
                self.mode = "TRACK"       # it was only a dropout
            elif self.advance_dist >= SONAR_AHEAD_M - LOST_CONFIRM_M:
                self.start_arc(ARC_RADIUS_M)
            else:
                self.steer(ARC_SPEED, 0.0)

        else:  # ARC
            if seen:
                self.mode = "TRACK"
                self.track_dist = 0.0
                self.lost_travel = 0.0
                self.prev_wall = None
                self.d_filtered = 0.0
                if DEBUG:
                    print("WALL FOUND {:.2f}".format(wall))
                self.track(wall, dt)
            elif (self.arc_start_heading - self.heading_cw
                  >= ARC_MAX_TURN_DEG):
                self.say("WALL LOST")
                self.begin_peel()         # nothing to follow: try to go home
            else:
                turn = ARC_SPEED * WHEEL_BASE_M / (2.0 * self.arc_radius)
                self.steer(ARC_SPEED, -turn)

    def loop_complete(self):
        net_left = self.first_follow_heading - self.heading_cw
        if self.follow_dist < MIN_LOOP_DIST_M or net_left < LOOP_MIN_TURN_DEG:
            return False
        if net_left >= LOOP_FORCE_TURN_DEG:
            return True
        d = self.dist_to(self.hit_x, self.hit_y)
        if d < self.loop_min_d:
            self.loop_min_d = d
        # Done once we have passed the closest approach to the hit point.
        return self.loop_min_d <= HIT_RADIUS_M and d >= self.loop_min_d + 0.03

    # ------------------------------ main loop ----------------------------

    def run(self):
        self.stop()
        self.say("Press CENTER")
        while Button.CENTER not in self.ev3.buttons.pressed():
            wait(20)
        wait(500)

        self.prev_left = self.left_motor.angle()
        self.prev_right = self.right_motor.angle()
        self.clock.reset()
        last_ms = 0
        star_x, star_y = MEASURE_AHEAD_M, 0.0   # where the star is

        while self.state != "DONE":
            wait(LOOP_MS)
            now = self.clock.time()
            dt = max(0.005, (now - last_ms) / 1000.0)
            last_ms = now
            if now > TIME_LIMIT_MS:
                self.say("TIME UP")
                break

            self.update_odometry()
            wall = self.read_wall()
            bumped = self.touch.pressed()

            if self.state == "DRIVE":
                turn = clamp(-KP_HEADING * self.heading_cw, -40, 40)
                self.steer(APPROACH_SPEED, turn)
                if bumped:
                    self.stop()
                    self.ev3.speaker.beep()
                    self.hit_x, self.hit_y = self.x, self.y
                    self.say("HIT")
                    self.begin_backup()

            elif self.state == "BACKUP":
                self.left_motor.run(-REVERSE_SPEED)
                self.right_motor.run(-REVERSE_SPEED)
                if (self.dist_to(self.bump_x, self.bump_y) >= BACKUP_DIST_M
                        or now - self.state_start_ms > 3000):
                    self.stop()
                    self.begin_turn(BUMP_TURN_DEG, "FOLLOW")

            elif self.state == "TURN":
                remaining = self.turn_target - self.heading_cw
                if (abs(remaining) <= 2.0
                        or now - self.state_start_ms > 6000):
                    self.stop()
                    wait(100)
                    self.begin_follow()
                else:
                    speed = clamp(abs(remaining) * 5.0, 40.0, TURN_SPEED)
                    if remaining > 0:
                        self.left_motor.run(speed)
                        self.right_motor.run(-speed)
                    else:
                        self.left_motor.run(-speed)
                        self.right_motor.run(speed)

            elif self.state == "FOLLOW":
                self.follow_dist += abs(self.d_center)

                # Stuck against something the bumper did not register.
                self.stuck_travel += abs(self.d_center)
                stuck = False
                if now - self.stuck_start_ms >= 1200:
                    stuck = self.stuck_travel < 0.015
                    self.stuck_start_ms = now
                    self.stuck_travel = 0.0

                if bumped or stuck:
                    self.begin_backup()
                    continue

                self.follow_step(wall, dt)
                if self.state != "FOLLOW":
                    continue

                if self.loop_complete():
                    self.stop()
                    self.ev3.speaker.beep()
                    self.say("BACK AT HIT")
                    self.begin_peel()
                    continue

                self.debug_counter += 1
                if DEBUG and self.debug_counter % 25 == 0:
                    print("{} wall={:.2f} hit={:.2f} trav={:.2f} left={:.0f}"
                          .format(self.mode, wall,
                                  self.dist_to(self.hit_x, self.hit_y),
                                  self.follow_dist,
                                  self.first_follow_heading
                                  - self.heading_cw))

            elif self.state == "PEEL":
                # Arc forward and RIGHT (away from the wall) until the robot
                # faces the start. No pivot next to the wall.
                if bumped:
                    self.say("PEEL BLOCKED")
                    break
                err = self.bearing_error_to(star_x, star_y)
                if abs(err) <= 25.0 or now - self.state_start_ms > 12000:
                    self.return_min_d = 99.0
                    self.set_state("RETURN")
                    self.say("RETURN")
                else:
                    self.steer(ARC_SPEED,
                               ARC_SPEED * WHEEL_BASE_M / (2.0 * 0.20))

            elif self.state == "RETURN":
                if bumped:
                    self.say("RETURN BLOCKED")
                    break
                # Stop when the MEASURING POINT (front of robot) is on the
                # star, i.e. the axle is MEASURE_AHEAD_M short of it.
                d = self.dist_to(star_x, star_y)
                if d < self.return_min_d:
                    self.return_min_d = d
                passed = self.return_min_d < 0.30 and d > self.return_min_d + 0.01
                if d <= MEASURE_AHEAD_M + 0.005 or passed:
                    self.stop()
                    self.ev3.speaker.beep()
                    self.say("AT START")
                    break
                err = self.bearing_error_to(star_x, star_y)
                speed = clamp((d - MEASURE_AHEAD_M) * 600.0, 70.0,
                              RETURN_SPEED)
                turn = clamp(2.5 * err, -0.4 * speed, 0.4 * speed)
                self.steer(speed, turn)

        self.stop()


def main():
    robot = Robot()
    try:
        robot.run()
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
