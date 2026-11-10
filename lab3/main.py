#!/usr/bin/env pybricks-micropython
# COMP 581 Lab 3 
# Caitlin Estrada (730661469) and Madeline Clark (730650946)

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch
import math


# ============================ HEADING SOURCE =============================
# Everything (wall angle, finding the hit point again, driving home) depends
# on heading. Encoder heading is only right if WHEEL_BASE_M is right FOR
# THIS FLOOR.
#   1. Set CALIBRATE_SPIN = True and run: the robot spins what it believes
#      is one full turn to the right, then stops.
#   2. If it really turned A degrees, set
#          WHEEL_BASE_M = WHEEL_BASE_M * 360 / A
#      (short of a full turn -> make it bigger; past it -> smaller).
#   3. Repeat until it lands on 360, then set CALIBRATE_SPIN = False.
# USE_GYRO = True uses the gyro on S1 for heading instead; flip GYRO_SIGN
# to -1 if a RIGHT turn makes the gyro angle go down.
CALIBRATE_SPIN = False
USE_GYRO = False
GYRO_SIGN = 1

# ====================== MEASURE THESE ON YOUR ROBOT ======================
WHEEL_RADIUS_M = 0.03175    # wheel radius
WHEEL_BASE_M = 0.127        # distance between the two wheel contact patches
SONAR_AHEAD_M = -0.06       # sonar position along the robot, measured from
                            # the wheel axle: + = in front, - = BEHIND it
SONAR_SIDE_M = 0.075        # robot centerline to the sonar's front face
MEASURE_AHEAD_M = 0.06      # axle to measuring point (tip of the bumper)

# ============================ WALL FOLLOWING =============================
TARGET_DIST_M = 0.10        # desired sonar reading while following
LOST_DIST_M = 0.30          # reading above this = wall not visible
TOO_CLOSE_M = 0.06          # below this always steer away
FOLLOW_SPEED = 220.0        # wheel deg/s (about 12 cm/s)
ARC_SPEED = 180.0           # wheel deg/s while rounding an outside corner
K_ANGLE = 2.4               # how hard to correct the angle to the wall (1/s)
K_DIST = 7.0                # approach angle (rad) per meter of distance error
MAX_APPROACH_DEG = 20.0     # never angle in/out more steeply than this
EST_DIST_GAIN = 0.12        # sonar correction of the distance estimate
EST_ANGLE_GAIN = 250.0      # sonar correction of the angle estimate
ACQUIRE_ANGLE_DEG = 30.0    # assumed angle away from a wall first seen in ARC
LOST_CONFIRM_M = 0.02       # travel with no wall before reacting
MIN_TRACK_M = 0.10          # wall must be tracked this far to use ADVANCE
ARC_BLIND_DEG = 20.0        # ignore the sonar for the first part of an arc
                            # (it can still see the wall just left behind)
ARC_MAX_TURN_DEG = 330.0    # give up if a full circle finds no wall

# ============================ OTHER MOTIONS ==============================
APPROACH_SPEED = 220.0
REVERSE_SPEED = 150.0
TURN_SPEED = 200.0
RETURN_SPEED = 220.0
BACKUP_DIST_M = 0.08
BUMP_TURN_DEG = 90.0        # right turn after a head-on bumper hit
GLANCE_TURN_DEG = 35.0      # smaller right turn after scraping the side wall
GLANCE_DIST_M = 0.075       # sonar closer than this at a bump = scraping
STUCK_MS = 1200             # no movement for this long counts as a bump
KP_HEADING = 3.0
TRIM = 1.5                  # small right-wheel speed correction

# =========================== LOOP COMPLETION =============================
MIN_LOOP_DIST_M = 1.20      # minimum wall-following travel
LOOP_MIN_TURN_DEG = 250.0   # net LEFT turn required before checking hit point
LOOP_FORCE_TURN_DEG = 450.0 # odometry drifted badly: call the loop done anyway
HIT_RADIUS_M = 0.45         # closest approach must be inside this radius

TIME_LIMIT_MS = 118000
LOOP_MS = 20
DEBUG = True
# =========================================================================

# Distance from the wall to the middle of the axle when following correctly.
AXLE_TARGET_M = TARGET_DIST_M + SONAR_SIDE_M
# Radius of the left arc around an outside corner. A sonar behind the axle
# only sees the next wall once it has travelled past the corner, so that
# layout needs a wider arc.
ARC_RADIUS_M = AXLE_TARGET_M + max(0.0, -SONAR_AHEAD_M - 0.02)
# How far the axle must move (+ forward, - backward) to sit just short of
# the corner once the wall has been lost for LOST_CONFIRM_M.
CORNER_SHIFT_M = SONAR_AHEAD_M - LOST_CONFIRM_M - (0.02 if SONAR_AHEAD_M < 0
                                                   else 0.0)
# Wheel deg/s of steering that turns the robot body at 1 rad/s.
STEER_PER_RAD_S = math.degrees(WHEEL_BASE_M / (2.0 * WHEEL_RADIUS_M))


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
        self.gyro = GyroSensor(Port.S1) if USE_GYRO else None
        self.clock = StopWatch()

        # Pose from wheel encoders. heading_cw is continuous (not wrapped),
        # in degrees, positive when the robot has turned RIGHT.
        self.x = 0.0
        self.y = 0.0
        self.heading_cw = 0.0
        self.d_center = 0.0     # forward travel during the last loop
        self.d_heading = 0.0    # heading change during the last loop
        self.prev_left = 0
        self.prev_right = 0

        self.state = "DRIVE"
        self.hit_x = 0.0
        self.hit_y = 0.0
        self.bump_x = 0.0
        self.bump_y = 0.0
        self.state_start_ms = 0

        self.turn_target = 0.0
        self.bump_turn = BUMP_TURN_DEG
        self.after_turn = "FOLLOW"

        # Wall following
        self.mode = "TRACK"
        self.readings = [LOST_DIST_M, LOST_DIST_M, LOST_DIST_M]
        self.wall_d = AXLE_TARGET_M   # estimated axle-to-wall distance
        self.wall_phi = 0.0           # estimated angle, + = nose toward wall
        self.track_dist = 0.0
        self.lost_travel = 0.0
        self.shift_dist = 0.0
        self.arc_start_heading = 0.0
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
        if USE_GYRO:
            self.d_heading = GYRO_SIGN * self.gyro.angle() - self.heading_cw
        else:
            self.d_heading = math.degrees((d_l - d_r) / WHEEL_BASE_M)
        mid = -math.radians(self.heading_cw + self.d_heading / 2.0)
        self.heading_cw += self.d_heading
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
        self.stuck_start_ms = self.state_start_ms
        self.stuck_travel = 0.0

    def is_stuck(self, now):
        """True if the wheels have barely moved for STUCK_MS."""
        self.stuck_travel += abs(self.d_center)
        if now - self.stuck_start_ms < STUCK_MS:
            return False
        stuck = self.stuck_travel < 0.015
        self.stuck_start_ms = now
        self.stuck_travel = 0.0
        return stuck

    def begin_backup(self, turn_degrees, next_state):
        self.stop()
        self.bump_turn = turn_degrees
        self.after_turn = next_state
        self.bump_x, self.bump_y = self.x, self.y
        self.set_state("BACKUP")

    def begin_follow(self):
        if self.first_follow_heading is None:
            self.first_follow_heading = self.heading_cw
        self.mode = "TRACK"
        self.track_dist = 0.0
        self.lost_travel = 0.0
        for _ in range(3):
            wall = self.read_wall()
            wait(15)
        self.reset_wall_estimate(wall, 0.0)
        self.set_state("FOLLOW")
        self.say("FOLLOW")

    def begin_peel(self):
        self.set_state("PEEL")
        self.say("PEEL")

    # --------------------------- wall following --------------------------

    def reset_wall_estimate(self, wall, phi):
        """Start the distance/angle estimate from one sonar reading."""
        wall = min(wall, LOST_DIST_M)
        self.wall_phi = phi
        self.wall_d = ((wall + SONAR_SIDE_M) * math.cos(phi)
                       + SONAR_AHEAD_M * math.sin(phi))

    def update_wall_estimate(self, wall):
        """Track distance (axle to wall) and angle (+ = nose toward wall).

        Prediction uses the wheel encoders: a left turn increases the angle,
        and driving at an angle changes the distance. The sonar then nudges
        both: a wall that is closer than predicted means the robot is nearer
        AND is angled further toward the wall than it thought.
        """
        self.wall_phi -= math.radians(self.d_heading)
        self.wall_d -= self.d_center * math.sin(self.wall_phi)
        c = math.cos(self.wall_phi)
        if c > 0.3:
            predicted = ((self.wall_d - SONAR_AHEAD_M * math.sin(self.wall_phi))
                         / c - SONAR_SIDE_M)
            surprise = clamp(wall - predicted, -0.05, 0.05)
            self.wall_d += EST_DIST_GAIN * surprise * c
            self.wall_phi -= EST_ANGLE_GAIN * surprise * abs(self.d_center)
        self.wall_phi = clamp(self.wall_phi, -1.2, 1.2)

    def track(self, wall):
        """Steer to hold the axle AXLE_TARGET_M from the wall on the LEFT."""
        self.update_wall_estimate(wall)
        limit = math.radians(MAX_APPROACH_DEG)
        # Too far -> aim the nose toward the wall, too close -> away from it.
        wanted_phi = clamp(K_DIST * (self.wall_d - AXLE_TARGET_M),
                           -limit, limit)
        turn_left = K_ANGLE * (wanted_phi - self.wall_phi)     # rad/s
        turn = -turn_left * STEER_PER_RAD_S
        # Never curve left tighter than the corner arc; allow a harder right.
        left_limit = FOLLOW_SPEED * WHEEL_BASE_M / (2.0 * ARC_RADIUS_M)
        turn = clamp(turn, -left_limit, 0.5 * FOLLOW_SPEED)
        if wall < TOO_CLOSE_M:
            turn = max(turn, 0.3 * FOLLOW_SPEED)
        self.steer(FOLLOW_SPEED, turn)

    def start_arc(self):
        self.mode = "ARC"
        self.arc_start_heading = self.heading_cw
        if DEBUG:
            print("ARC")

    def follow_step(self, wall):
        seen = wall <= LOST_DIST_M
        step = abs(self.d_center)

        if self.mode == "TRACK":
            if seen:
                self.lost_travel = 0.0
                self.track_dist += step
                self.track(wall)
            else:
                self.lost_travel += step
                if self.lost_travel < LOST_CONFIRM_M:
                    self.steer(ARC_SPEED, 0.0)
                elif self.track_dist >= MIN_TRACK_M and CORNER_SHIFT_M > 0:
                    self.mode = "ADVANCE"
                    self.shift_dist = 0.0
                    if DEBUG:
                        print("ADVANCE")
                elif self.track_dist >= MIN_TRACK_M and CORNER_SHIFT_M < 0:
                    self.mode = "REVERSE"
                    self.shift_dist = 0.0
                    if DEBUG:
                        print("REVERSE")
                else:
                    self.start_arc()

        elif self.mode == "ADVANCE":
            self.shift_dist += step
            if seen:
                self.mode = "TRACK"       # it was only a dropout
            elif self.shift_dist >= CORNER_SHIFT_M:
                self.start_arc()
            else:
                self.steer(ARC_SPEED, 0.0)

        elif self.mode == "REVERSE":
            # The sonar sees the old wall again while backing up: ignore it.
            self.shift_dist += step
            if self.shift_dist >= -CORNER_SHIFT_M:
                self.stop()
                self.start_arc()
            else:
                self.left_motor.run(-REVERSE_SPEED)
                self.right_motor.run(-REVERSE_SPEED)

        else:  # ARC
            turned = self.arc_start_heading - self.heading_cw
            if seen and turned >= ARC_BLIND_DEG:
                self.mode = "TRACK"
                self.track_dist = 0.0
                self.lost_travel = 0.0
                # The sonar first catches the new wall while the robot is
                # still angled away from it.
                self.reset_wall_estimate(
                    wall, -math.radians(ACQUIRE_ANGLE_DEG))
                if DEBUG:
                    print("WALL FOUND {:.2f}".format(wall))
                self.track(wall)
            elif turned >= ARC_MAX_TURN_DEG:
                self.say("WALL LOST")
                self.begin_peel()         # nothing to follow: try to go home
            else:
                turn = ARC_SPEED * WHEEL_BASE_M / (2.0 * ARC_RADIUS_M)
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
        if USE_GYRO:
            self.gyro.reset_angle(0)
        self.clock.reset()

        if CALIBRATE_SPIN:
            # Spin what the robot believes is exactly 360 degrees right.
            while self.heading_cw < 360.0 and self.clock.time() < 20000:
                speed = clamp((360.0 - self.heading_cw) * 5.0, 40.0, 150.0)
                self.left_motor.run(speed)
                self.right_motor.run(-speed)
                wait(LOOP_MS)
                self.update_odometry()
            self.stop()
            self.say("SPUN 360?")
            wait(5000)
            return

        star_x, star_y = MEASURE_AHEAD_M, 0.0   # where the star is
        self.set_state("DRIVE")

        while self.state != "DONE":
            wait(LOOP_MS)
            now = self.clock.time()
            if now > TIME_LIMIT_MS:
                self.say("TIME UP")
                break

            self.update_odometry()
            wall = self.read_wall()
            bumped = self.touch.pressed()

            if self.state == "DRIVE":
                turn = clamp(-KP_HEADING * self.heading_cw, -40, 40)
                self.steer(APPROACH_SPEED, turn)
                # The bumper is narrow: an angled wall can stop the robot
                # without pressing it, so a stall also counts as the hit.
                stuck = self.is_stuck(now)
                if bumped or stuck:
                    self.stop()
                    self.ev3.speaker.beep()
                    self.hit_x, self.hit_y = self.x, self.y
                    self.say("HIT")
                    self.begin_backup(BUMP_TURN_DEG, "FOLLOW")

            elif self.state == "BACKUP":
                self.left_motor.run(-REVERSE_SPEED)
                self.right_motor.run(-REVERSE_SPEED)
                if (self.dist_to(self.bump_x, self.bump_y) >= BACKUP_DIST_M
                        or now - self.state_start_ms > 3000):
                    self.stop()
                    self.turn_target = self.heading_cw + self.bump_turn
                    self.set_state("TURN")

            elif self.state == "TURN":
                remaining = self.turn_target - self.heading_cw
                if (abs(remaining) <= 2.0
                        or now - self.state_start_ms > 6000):
                    self.stop()
                    wait(100)
                    if self.after_turn == "FOLLOW":
                        self.begin_follow()
                    else:
                        self.return_min_d = 99.0
                        self.set_state("RETURN")
                        self.say("RETURN")
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

                stuck = self.is_stuck(now)
                if bumped or stuck:
                    # Scraping the wall we are following needs only a small
                    # turn away; a wall straight ahead needs the full 90.
                    glancing = self.mode == "TRACK" and wall <= GLANCE_DIST_M
                    if DEBUG:
                        print("BUMP wall={:.2f} glancing={}".format(
                            wall, glancing))
                    self.begin_backup(GLANCE_TURN_DEG if glancing
                                      else BUMP_TURN_DEG, "FOLLOW")
                    continue

                self.follow_step(wall)
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
                    print("{} t={:.0f} wall={:.2f} d={:.2f} ang={:.0f} "
                          "hit={:.2f} left={:.0f}"
                          .format(self.mode, now / 1000.0, wall, self.wall_d,
                                  math.degrees(self.wall_phi),
                                  self.dist_to(self.hit_x, self.hit_y),
                                  self.first_follow_heading
                                  - self.heading_cw))

            elif self.state == "PEEL":
                # Arc forward and RIGHT (away from the wall) until the robot
                # faces the start. No pivot next to the wall.
                err = self.bearing_error_to(star_x, star_y)
                stuck = self.is_stuck(now)
                if bumped or stuck:
                    # Something is in the way: back off, then pivot to face
                    # the start and go.
                    self.begin_backup(0.0, "RETURN")
                elif abs(err) <= 25.0 or now - self.state_start_ms > 12000:
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
                if abs(err) > 40.0:
                    # Facing well away from the start: pivot first.
                    speed = clamp(abs(err) * 4.0, 60.0, TURN_SPEED)
                    if err > 0:
                        self.left_motor.run(speed)
                        self.right_motor.run(-speed)
                    else:
                        self.left_motor.run(-speed)
                        self.right_motor.run(speed)
                else:
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
