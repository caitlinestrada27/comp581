#!/usr/bin/env pybricks-micropython
# COMP 581 Lab 3
# Caitlin Estrada (730661469) and Madeline Clark (730650946)
#
# Strategy
#   DRIVE   straight until the bumper hits the obstacle (beep, save hit point)
#   BACKUP  a few cm, TURN right 90 deg so the wall is on the robot's LEFT
#   FOLLOW  the wall with the left-facing ultrasonic sensor:
#             TRACK    wall visible -> steer using an estimate of the robot's
#                      distance AND angle to the wall. The angle comes from
#                      the wheel encoders, corrected slowly by the sonar.
#                      (Needed because the sonar is BEHIND the axle: steering
#                      on the raw sonar reading alone is unstable.)
#             CORNER   wall lost = the sonar just passed an outside corner.
#                      Remember where that corner is, then repeat:
#                        drive straight until the SONAR is past the corner
#                        along the current heading; if it sees a wall there,
#                        go back to TRACK; if not, pivot left one step.
#                      The next wall is found by sensing, not by bumping,
#                      whether the corner is gentle or sharp.
#           Bumper hit (inside corner) -> BACKUP, TURN right, FOLLOW again
#   Loop is complete when the robot has turned about a full circle to the
#   left AND has passed its closest approach to the saved hit point.
#   PEEL    arc right, away from the wall, until facing the start
#   RETURN  drive to the start and stop with the measuring point on the star
#
# Creative contribution: once the robot has STOPPED, it reports how the run
# went. Back at the start: green light, a face on the screen and a victory
# fanfare. Out of time or blocked: red light and a sad falling tune. Both
# tunes are our own, played one note at a time with speaker.beep().

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Button, Color
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
TARGET_DIST_M = 0.09        # desired sonar reading while following
LOST_DIST_M = 0.30          # reading above this = wall not visible
TOO_CLOSE_M = 0.06          # below this always steer away
FOLLOW_SPEED = 220.0        # wheel deg/s (about 12 cm/s)
# If a lap is too slow for the 2-minute limit, a faster set that also worked
# in simulation is FOLLOW_SPEED 270, ARC_SPEED 220, CORNER_TURN_SPEED 140.
ARC_SPEED = 180.0           # wheel deg/s during corner probes and the exit arc
K_ANGLE = 2.4               # how hard to correct the angle to the wall (1/s)
K_DIST = 4.0                # approach angle (rad) per meter of distance error
MAX_APPROACH_DEG = 10.0     # never angle in/out more steeply than this
                            # (the sonar goes blind at steep angles)
COAST_ANGLE_DEG = 12.0      # wall lost while angled more than this: assume
                            # the angle blinded the sonar, not a corner
COAST_MAX_M = 0.12          # ...and straighten up for at most this far
K_CURVE = 15.0              # learns a steady turn rate for curved walls
EST_DIST_GAIN = 0.12        # sonar correction of the distance estimate
EST_ANGLE_GAIN = 250.0      # sonar correction of the angle estimate
LOST_CONFIRM_M = 0.02       # travel with no wall before reacting

# ========================= OUTSIDE CORNER SEARCH =========================
CORNER_STEP_DEG = 30.0      # left pivot between straight probes
CORNER_PAST_M = 0.0         # sonar must be this far past the corner (along
                            # the current heading) before a reading counts
CORNER_CONFIRM_M = 0.04     # ...and the wall must stay visible this far
CORNER_TURN_SPEED = 110.0   # wheel deg/s while pivoting
CORNER_MAX_TURN_DEG = 300.0 # give up if nearly a full circle finds no wall

# ============================ OTHER MOTIONS ==============================
APPROACH_SPEED = 220.0
REVERSE_SPEED = 150.0
TURN_SPEED = 200.0
RETURN_SPEED = 220.0
BACKUP_DIST_M = 0.08
PEEL_TURN_DEG = 35.0        # arc this far right, away from the wall, then
                            # pivot in place to face the start
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

TIME_LIMIT_MS = 116000      # stop driving here; leaves time for the sad tune

# ================================ FINALE =================================
# (frequency in Hz, length in ms); frequency 0 is a rest.
VICTORY_TUNE = [(523, 120), (659, 120), (784, 120), (1047, 240), (0, 60),
                (784, 120), (1047, 420)]
SAD_TUNE = [(392, 260), (370, 260), (349, 260), (330, 600)]
LOOP_MS = 20
DEBUG = True
# =========================================================================

# Distance from the wall to the middle of the axle when following correctly.
AXLE_TARGET_M = TARGET_DIST_M + SONAR_SIDE_M
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
        self.curve_rate = 0.0         # learned steady left turn rate (rad/s)
        self.track_dist = 0.0
        self.lost_travel = 0.0
        self.last_wall = TARGET_DIST_M   # last sonar reading that saw a wall
        self.corner_x = 0.0           # where the wall ended (outside corner)
        self.corner_y = 0.0
        self.corner_start_heading = 0.0
        self.corner_pivoting = False
        self.pivot_target = 0.0
        self.seen_travel = 0.0
        self.seen_first = 0.0         # first reading of a newly seen wall
        self.follow_dist = 0.0
        self.first_follow_heading = None
        self.loop_min_d = 99.0

        self.stuck_start_ms = 0
        self.stuck_travel = 0.0
        self.return_min_d = 99.0
        self.peel_start_heading = 0.0
        self.debug_counter = 0
        self.outcome = "gave up"     # becomes "home" or "time up"

    # ------------------------------ helpers ------------------------------

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def steer(self, speed, turn):
        # Positive turn curves RIGHT; negative turn curves LEFT.
        self.left_motor.run(speed + turn)
        self.right_motor.run(speed - turn + TRIM)

    def finale(self):
        """Creative contribution: announce how the run ended.

        Called only after the motors have stopped, so it never affects the
        driving or the finishing position.
        """
        if self.outcome == "home":
            self.ev3.light.on(Color.GREEN)
            self.ev3.screen.clear()
            self.ev3.screen.print("  ^   ^")
            self.ev3.screen.print("  \\___/")
            self.ev3.screen.print("HOME!")
            tune = VICTORY_TUNE
        else:
            self.ev3.light.on(Color.RED)
            self.ev3.screen.clear()
            self.ev3.screen.print("  -   -")
            self.ev3.screen.print("  /```\\")
            self.ev3.screen.print(self.outcome)
            tune = SAD_TUNE
        for frequency, duration in tune:
            if frequency > 0:
                self.ev3.speaker.beep(frequency, duration)
            else:
                wait(duration)
            wait(30)
        wait(1500)

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
        # Count wheel movement, so pivoting in place is not "stuck".
        self.stuck_travel += (abs(self.d_center)
                              + math.radians(abs(self.d_heading))
                              * WHEEL_BASE_M / 2.0)
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
        self.peel_start_heading = self.heading_cw
        self.set_state("PEEL")
        self.say("PEEL")

    # --------------------------- wall following --------------------------

    def reset_wall_estimate(self, wall, phi):
        """Start the distance/angle estimate from one sonar reading."""
        wall = min(wall, LOST_DIST_M)
        self.wall_phi = phi
        self.curve_rate = 0.0
        self.wall_d = ((wall + SONAR_SIDE_M) * math.cos(phi)
                       + SONAR_AHEAD_M * math.sin(phi))

    def update_wall_estimate(self, wall):
        """Track distance (axle to wall) and angle (+ = nose toward wall).
        Pass wall=None when the sonar sees nothing (encoders only).

        Prediction uses the wheel encoders: a left turn increases the angle,
        and driving at an angle changes the distance. The sonar then nudges
        both: a wall that is closer than predicted means the robot is nearer
        AND is angled further toward the wall than it thought.
        """
        self.wall_phi -= math.radians(self.d_heading)
        self.wall_d -= self.d_center * math.sin(self.wall_phi)
        c = math.cos(self.wall_phi)
        if wall is not None and c > 0.3:
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
        # A wall that curves away needs a steady left turn on top of the
        # corrections; learn it from distance error that will not go away.
        self.curve_rate = clamp(
            self.curve_rate + K_CURVE * (self.wall_d - AXLE_TARGET_M)
            * abs(self.d_center), -0.2, 0.5)
        turn_left = (K_ANGLE * (wanted_phi - self.wall_phi)
                     + self.curve_rate)                        # rad/s
        turn = -turn_left * STEER_PER_RAD_S
        # Limit how tightly it curls toward the wall; allow a harder right.
        left_limit = FOLLOW_SPEED * WHEEL_BASE_M / (2.0 * AXLE_TARGET_M)
        turn = clamp(turn, -left_limit, 0.5 * FOLLOW_SPEED)
        if wall is not None and wall < TOO_CLOSE_M:
            turn = max(turn, 0.3 * FOLLOW_SPEED)
        self.steer(FOLLOW_SPEED, turn)

    def start_corner(self):
        """The wall just ended. Work out where its end (the corner) is."""
        a = -math.radians(self.heading_cw)
        ahead_x, ahead_y = math.cos(a), math.sin(a)       # robot's forward
        left_x, left_y = -math.sin(a), math.cos(a)        # robot's left
        # The sonar was this far along the robot when the wall vanished...
        along = SONAR_AHEAD_M - self.lost_travel
        # ...and the wall was this far to the left of the robot's centerline.
        if self.track_dist > 0.02:
            side = self.last_wall + SONAR_SIDE_M
        else:
            side = AXLE_TARGET_M
        self.corner_x = self.x + along * ahead_x + side * left_x
        self.corner_y = self.y + along * ahead_y + side * left_y
        self.corner_start_heading = self.heading_cw
        self.corner_pivoting = False
        self.seen_travel = 0.0
        self.mode = "CORNER"
        if DEBUG:
            print("CORNER")

    def corner_step(self, wall, seen, step):
        turned = self.corner_start_heading - self.heading_cw

        if self.corner_pivoting:
            remaining = self.pivot_target - turned
            if remaining <= 1.5:
                self.corner_pivoting = False
                self.seen_travel = 0.0
                self.stop()
            else:
                speed = clamp(remaining * 6.0, 40.0, CORNER_TURN_SPEED)
                self.left_motor.run(-speed)
                self.right_motor.run(speed)
            return

        # Straight probe. How far is the SONAR past the corner, measured
        # along the direction the robot is now facing? Until it is past,
        # the next wall cannot be in front of the sonar, and anything it
        # hears is the old wall or the corner edge.
        a = -math.radians(self.heading_cw)
        past = ((self.x - self.corner_x) * math.cos(a)
                + (self.y - self.corner_y) * math.sin(a) + SONAR_AHEAD_M)
        if seen:
            if self.seen_travel == 0.0:
                self.seen_first = wall
            self.seen_travel += step
        else:
            self.seen_travel = 0.0
        confirmed = self.seen_travel >= CORNER_CONFIRM_M

        if confirmed and turned < 5.0:
            self.mode = "TRACK"           # same wall: it was only a dropout
            self.lost_travel = 0.0
            self.track(wall)
        elif confirmed and past >= CORNER_PAST_M:
            self.mode = "TRACK"
            self.track_dist = 0.0
            self.lost_travel = 0.0
            # Driving straight, the reading shrinks if the robot is angled
            # toward the new wall and grows if angled away: that slope
            # gives the starting angle estimate.
            slope = (wall - self.seen_first) / self.seen_travel
            phi = clamp(math.atan(-slope), -0.5, 0.5)
            self.reset_wall_estimate(wall, phi)
            if DEBUG:
                print("WALL FOUND {:.2f} after {:.0f} deg, angle {:.0f}"
                      .format(wall, turned, math.degrees(phi)))
            self.track(wall)
        elif past >= CORNER_PAST_M + 0.01 and not seen:
            if turned >= CORNER_MAX_TURN_DEG:
                self.say("WALL LOST")
                self.begin_peel()         # nothing to follow: try to go home
            else:
                # Next multiple of the step, so small overshoots do not add up.
                self.corner_pivoting = True
                self.pivot_target = CORNER_STEP_DEG * (
                    int((turned + 5.0) / CORNER_STEP_DEG) + 1)
        else:
            self.steer(ARC_SPEED, 0.0)

    def follow_step(self, wall):
        seen = wall <= LOST_DIST_M
        step = abs(self.d_center)

        if self.mode == "TRACK":
            if seen:
                self.lost_travel = 0.0
                self.track_dist += step
                self.last_wall = wall
                self.track(wall)
            else:
                self.lost_travel += step
                angled = abs(self.wall_phi) > math.radians(COAST_ANGLE_DEG)
                if angled and self.lost_travel < COAST_MAX_M:
                    # Probably blinded by the angle to the wall, not a
                    # corner: steer back to parallel on the estimate alone.
                    self.track(None)
                elif self.lost_travel < LOST_CONFIRM_M:
                    self.track(None)
                else:
                    self.start_corner()
        else:
            self.corner_step(wall, seen, step)

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
                self.outcome = "time up"
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
                # Arc forward and RIGHT a little to get clear of the wall;
                # RETURN then pivots in place to face the start.
                err = self.bearing_error_to(star_x, star_y)
                stuck = self.is_stuck(now)
                if bumped or stuck:
                    # Something is in the way: back off, then pivot to face
                    # the start and go.
                    self.begin_backup(0.0, "RETURN")
                elif (abs(err) <= 25.0
                      or self.heading_cw - self.peel_start_heading
                      >= PEEL_TURN_DEG
                      or now - self.state_start_ms > 12000):
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
                    self.say("AT START")
                    self.outcome = "home"
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
        self.finale()


def main():
    robot = Robot()
    try:
        robot.run()
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
