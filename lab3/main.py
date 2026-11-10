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
#   RETURN  drive back to where the program was started, then turn to face
#           the original direction (measuring point back on the start)
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
# USE_GYRO = True takes heading from the gyro on S1 instead, which does not
# depend on wheel slip or WHEEL_BASE_M. Its direction is detected by the
# program during the first turn; if the gyro is missing or dead the program
# falls back to the wheel encoders. KEEP THE ROBOT STILL when the program
# starts, or the gyro drifts.
CALIBRATE_SPIN = False
USE_GYRO = True

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
FOLLOW_SPEED = 260.0        # wheel deg/s (about 12 cm/s)
# If a lap is too slow for the 2-minute limit, a faster set that also worked
# in simulation is FOLLOW_SPEED 270, ARC_SPEED 220, CORNER_TURN_SPEED 140.
ARC_SPEED = 210.0           # wheel deg/s during corner probes and the exit arc
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
CORNER_TURN_SPEED = 160.0   # wheel deg/s while pivoting
CORNER_MAX_TURN_DEG = 300.0 # give up if nearly a full circle finds no wall

# ============================ OTHER MOTIONS ==============================
APPROACH_SPEED = 260.0
REVERSE_SPEED = 150.0
TURN_SPEED = 200.0
RETURN_SPEED = 260.0
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
LOOP_MIN_TURN_DEG = 300.0   # net LEFT turn required before checking hit point
LOOP_MIN_TURN_ENC_DEG = 250.0  # same, when heading comes from the encoders
LOOP_LATERAL_M = 0.80       # allowed sideways position drift at the hit point
RELOCALIZE = True           # correct heading/position off the first wall
                            # when the lap closes (False = pure odometry)
MAX_DETOURS = 4             # times to go around something blocking the way home
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
        self.gyro = None
        if USE_GYRO:
            try:
                self.gyro = GyroSensor(Port.S1)
            except Exception:
                print("NO GYRO ON S1 - using wheel encoders")
        self.gyro_sign = 0      # 0 = gyro not trusted yet, else +1 / -1
        self.gyro_prev = 0
        self.gyro_turn0 = 0
        self.enc_turn = 0.0     # total turn according to the wheel encoders
        self.enc_turn0 = 0.0
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
        self.pivot_start_ms = 0
        self.seen_travel = 0.0
        self.seen_first = 0.0         # first reading of a newly seen wall
        self.follow_dist = 0.0
        self.first_follow_heading = None
        self.loop_min_d = 99.0

        self.stuck_start_ms = 0
        self.stuck_travel = 0.0
        self.return_min_d = 99.0
        self.peel_start_heading = 0.0
        self.approach_len = 0.0       # start-to-obstacle distance driven
        self.return_travel = 0.0      # driven since leaving the wall
        self.debug_counter = 0
        self.outcome = "gave up"     # becomes "home" or "time up"
        self.homing = False           # following a wall to get around it
        self.homing_start = 0.0
        self.detours = 0
        # Re-localisation against the first wall (see relocalize()).
        self.tip_x = 0.0              # where the bumper touched the wall
        self.tip_y = 0.0
        self.on_first_wall = False
        self.wall1_sum = 0.0          # first pass: heading when parallel
        self.wall1_n = 0
        self.par_avg = 0.0            # recent heading-when-parallel
        self.par_n = 0

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
        enc_d = math.degrees((d_l - d_r) / WHEEL_BASE_M)
        self.enc_turn += enc_d
        if self.gyro_sign != 0:
            raw = self.gyro.angle()
            self.d_heading = self.gyro_sign * (raw - self.gyro_prev)
            self.gyro_prev = raw
        else:
            self.d_heading = enc_d
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
        self.return_travel = 0.0
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

    def relocalize(self):
        """The lap has closed, so the robot is beside the FIRST wall again.

        Two things are known exactly there, whatever the odometry says:
        the wall's direction (the same as on the first pass, one full turn
        ago) and the robot's distance from it (the sonar). Use them to
        remove heading drift and position drift across the wall before
        driving home.
        """
        if not RELOCALIZE:
            return
        # First wall's direction: measured on the first pass if possible,
        # otherwise the heading the robot turned to after the hit.
        if self.wall1_n >= 10:
            wall_heading = self.wall1_sum / self.wall1_n
        else:
            wall_heading = self.first_follow_heading
        # Robot's heading-when-parallel and wall distance right now.
        if self.mode == "TRACK" and self.par_n >= 8:
            parallel_now = self.par_avg
            wall_dist = self.wall_d
        else:
            parallel_now = self.heading_cw
            wall_dist = AXLE_TARGET_M
        heading_error = parallel_now - (wall_heading - 360.0)
        a = -math.radians(wall_heading)
        away_x, away_y = math.sin(a), -math.cos(a)    # pointing off the wall
        drift = ((self.x - self.tip_x) * away_x
                 + (self.y - self.tip_y) * away_y - wall_dist)
        print("RELOCALIZE heading_error={:.1f} drift={:.2f}".format(
            heading_error, drift))
        # The wall direction and the sonar distance are measurements, so
        # trust them completely, however far the odometry has drifted.
        if abs(heading_error) <= 40.0:
            self.heading_cw -= heading_error
        self.x -= drift * away_x
        self.y -= drift * away_y
        # Along the wall the robot is known to be just past the hit point.
        along_x, along_y = math.cos(a), math.sin(a)
        along = ((self.x - self.tip_x) * along_x
                 + (self.y - self.tip_y) * along_y)
        wanted = clamp(along, 0.0, 0.20)
        self.x += (wanted - along) * along_x
        self.y += (wanted - along) * along_y

    def start_corner(self):
        self.on_first_wall = False
        self.par_n = 0
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
            if (remaining <= 2.0
                    or self.clock.time() - self.pivot_start_ms > 2500):
                self.corner_pivoting = False
                self.seen_travel = 0.0
                self.stop()
            else:
                # Constant firm speed: slow pivots stall on carpet.
                self.left_motor.run(-CORNER_TURN_SPEED)
                self.right_motor.run(CORNER_TURN_SPEED)
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
                self.pivot_start_ms = self.clock.time()
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
                # Heading the robot would have if exactly parallel to the wall.
                parallel = self.heading_cw + math.degrees(self.wall_phi)
                self.par_n += 1
                if self.par_n == 1:
                    self.par_avg = parallel
                else:
                    self.par_avg += 0.08 * (parallel - self.par_avg)
                if self.on_first_wall and 0.08 <= self.track_dist <= 0.30:
                    self.wall1_sum += parallel
                    self.wall1_n += 1
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
        min_turn = (LOOP_MIN_TURN_DEG if self.gyro_sign != 0
                    else LOOP_MIN_TURN_ENC_DEG)
        if self.follow_dist < MIN_LOOP_DIST_M or net_left < min_turn:
            return False
        if net_left >= LOOP_FORCE_TURN_DEG:
            return True
        # 1) Passed the closest approach to the hit point.
        d = self.dist_to(self.hit_x, self.hit_y)
        if d < self.loop_min_d:
            self.loop_min_d = d
        if self.loop_min_d <= HIT_RADIUS_M and d >= self.loop_min_d + 0.03:
            return True
        # 2) Back on the first wall (almost a full turn made) and level
        #    with the hit point along it, even if position drifted sideways.
        if self.gyro_sign == 0:
            return False
        a = -math.radians(self.first_follow_heading)
        dx = self.x - self.hit_x
        dy = self.y - self.hit_y
        along = dx * math.cos(a) + dy * math.sin(a)
        lateral = -dx * math.sin(a) + dy * math.cos(a)
        return along >= 0.0 and abs(lateral) <= LOOP_LATERAL_M

    # ------------------------------ main loop ----------------------------

    def run(self):
        self.stop()
        self.say("Press CENTER")
        while Button.CENTER not in self.ev3.buttons.pressed():
            wait(20)
        wait(500)

        self.prev_left = self.left_motor.angle()
        self.prev_right = self.right_motor.angle()
        if self.gyro is not None:
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

        # Home = where the wheel axle was when the button was pressed.
        # Driving the axle back there and then facing the original way
        # puts the measuring point back on the start point.
        star_x, star_y = 0.0, 0.0
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
                    self.approach_len = self.dist_to(0.0, 0.0)
                    a = -math.radians(self.heading_cw)
                    self.tip_x = self.x + MEASURE_AHEAD_M * math.cos(a)
                    self.tip_y = self.y + MEASURE_AHEAD_M * math.sin(a)
                    self.on_first_wall = True
                    self.say("HIT")
                    self.begin_backup(BUMP_TURN_DEG, "FOLLOW")

            elif self.state == "BACKUP":
                self.left_motor.run(-REVERSE_SPEED)
                self.right_motor.run(-REVERSE_SPEED)
                if (self.dist_to(self.bump_x, self.bump_y) >= BACKUP_DIST_M
                        or now - self.state_start_ms > 3000):
                    self.stop()
                    self.turn_target = self.heading_cw + self.bump_turn
                    self.enc_turn0 = self.enc_turn
                    if self.gyro is not None:
                        self.gyro_turn0 = self.gyro.angle()
                    self.set_state("TURN")

            elif self.state == "TURN":
                if self.gyro is not None and self.gyro_sign == 0:
                    # Compare gyro and encoders over the first big turn to
                    # learn which way the gyro counts, then switch to it.
                    enc = self.enc_turn - self.enc_turn0
                    if abs(enc) >= 45.0:
                        raw = self.gyro.angle()
                        gyro = raw - self.gyro_turn0
                        if abs(gyro) >= 20.0:
                            self.gyro_sign = 1 if gyro * enc > 0 else -1
                            self.gyro_prev = raw
                            self.heading_cw += self.gyro_sign * gyro - enc
                            print("GYRO OK sign={} gyro={} enc={:.0f}".format(
                                self.gyro_sign, gyro, enc))
                        else:
                            self.gyro = None
                            print("GYRO NOT RESPONDING - using encoders")
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
                    speed = clamp(abs(remaining) * 5.0, 70.0, TURN_SPEED)
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
                    self.on_first_wall = False
                    self.par_n = 0
                    if DEBUG:
                        print("BUMP wall={:.2f} glancing={}".format(
                            wall, glancing))
                    self.begin_backup(GLANCE_TURN_DEG if glancing
                                      else BUMP_TURN_DEG, "FOLLOW")
                    continue

                self.follow_step(wall)
                if self.state != "FOLLOW":
                    continue

                if self.homing:
                    # Going around something that blocked the way home:
                    # leave the wall once the start is off to the right.
                    err = self.bearing_error_to(star_x, star_y)
                    if (self.follow_dist - self.homing_start >= 0.25
                            and 20.0 <= err <= 160.0):
                        self.homing = False
                        self.begin_peel()
                    continue

                if self.loop_complete():
                    self.stop()
                    self.relocalize()
                    self.ev3.speaker.beep()
                    self.say("HIT POINT FOUND")
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
                stuck = self.is_stuck(now)
                if bumped or stuck:
                    # Something is between the robot and the start: follow
                    # its wall for a while, then head home again.
                    self.detours += 1
                    if self.detours > MAX_DETOURS:
                        self.say("RETURN BLOCKED")
                        break
                    self.say("DETOUR")
                    self.homing = True
                    self.homing_start = self.follow_dist
                    self.begin_backup(BUMP_TURN_DEG, "FOLLOW")
                    continue
                d = self.dist_to(star_x, star_y)
                err = self.bearing_error_to(star_x, star_y)
                if abs(err) > 40.0 and d > 0.06:
                    # Facing well away from home: pivot first. (Distance
                    # is not judged while pivoting.)
                    speed = clamp(abs(err) * 4.0, 80.0, TURN_SPEED)
                    if err > 0:
                        self.left_motor.run(speed)
                        self.right_motor.run(-speed)
                    else:
                        self.left_motor.run(-speed)
                        self.right_motor.run(speed)
                    self.return_min_d = 99.0
                else:
                    if d < self.return_min_d:
                        self.return_min_d = d
                    passed = (self.return_min_d < 0.30
                              and d > self.return_min_d + 0.02)
                    # Safety cap: home can never be farther from the wall
                    # than the original approach, so never drive farther
                    # than that (keeps the robot inside the workspace).
                    self.return_travel += abs(self.d_center)
                    too_far = self.return_travel >= self.approach_len + 0.05
                    if d <= 0.015 or passed or too_far:
                        self.stop()
                        self.set_state("FACE")
                        self.say("FACE")
                    else:
                        speed = clamp(d * 600.0, 70.0, RETURN_SPEED)
                        turn = clamp(2.5 * err, -0.4 * speed, 0.4 * speed)
                        self.steer(speed, turn)

            elif self.state == "FACE":
                # At home: turn to face the way the robot started.
                remaining = wrap_degrees(-self.heading_cw)
                if (abs(remaining) <= 3.0
                        or now - self.state_start_ms > 6000):
                    self.stop()
                    self.say("AT START")
                    self.outcome = "home"
                    break
                speed = clamp(abs(remaining) * 4.0, 80.0, TURN_SPEED)
                if remaining > 0:
                    self.left_motor.run(speed)
                    self.right_motor.run(-speed)
                else:
                    self.left_motor.run(-speed)
                    self.right_motor.run(speed)

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
