#!/usr/bin/env pybricks-micropython
# COMP 581 Lab 4: Bug Race
# Caitlin Estrada (730661469) and Madeline Clark (730650946)
#
# Strategy (a "Bug" algorithm built on our Lab 3 wall follower)
#   The robot keeps track of where it is (x, y in the lab's coordinates)
#   from the wheel encoders, with heading from the gyro.
#   GO      face the goal and drive straight at it. Stop and beep when the
#           measuring point (bumper tip) reaches the goal point.
#   Either bumper (left on S2, right on S3) hit on the way (the "hit point") -> remember how far the goal
#           was, BACKUP, then TURN right until the sonar sees the wall on
#           the LEFT (at most 90 deg).
#   Robot stalled without the bumper being pressed -> its side is scraping
#           a wall: BACKUP and, if the sonar sees that wall on the left,
#           start following it without turning.
#   FOLLOW  the wall with the left-facing ultrasonic sensor (Lab 3 code):
#             TRACK    wall visible -> steer using an estimate of the robot's
#                      distance AND angle to the wall.
#             CORNER   wall lost = the sonar just passed an outside corner:
#                      drive straight until the sonar is past the corner; if
#                      it sees a wall, TRACK it; if not, pivot left one step.
#           Bumper hit (inside corner) -> BACKUP, TURN right, FOLLOW again.
#   LEAVE the wall and GO again as soon as BOTH are true:
#             1. the robot is closer to the goal than it was at the hit point
#             2. the goal is off to the robot's RIGHT (the wall is on its
#                left, so that direction is clear of this wall)
#           Rule 1 means every hit point is closer to the goal than the one
#           before, so the robot cannot circle between obstacles forever.
#   Safety nets: if it goes all the way round an obstacle without being able
#   to leave, it stops at the closest point to the goal it found; and it
#   always stops by itself before the 3-minute limit.

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Button, Color
from pybricks.tools import wait, StopWatch
import math


# ================================ THE TASK ===============================
# Lab coordinates in meters. The start line is the x axis and the robot
# starts behind it facing +y (straight into the workspace).
START_X_M = 0.50            # where the MEASURING POINT (bumper tip) starts
START_Y_M = 0.00
START_THETA_DEG = 90.0      # direction the robot faces at the start,
                            # counter-clockwise from the +x axis
GOAL_X_M = 2.50
GOAL_Y_M = 2.50
# Aim correction. Measured on the robot: it stops to the RIGHT of the goal
# (too far in +x) and SHORT of it (not far enough in +y), by 26-28 cm.
# So it aims at a point shifted the other way: left and further forward.
# To set from measurements: AIM_SHIFT = goal - where it really stopped,
# averaged over a few runs. Set both to 0.0 to aim at the goal exactly.
AIM_SHIFT_X_M = 0.0         # tuned at fast speed; reset for normal speed
AIM_SHIFT_Y_M = 0.0
AIM_X_M = GOAL_X_M + AIM_SHIFT_X_M     # the point the robot drives to
AIM_Y_M = GOAL_Y_M + AIM_SHIFT_Y_M

# ============================== CALIBRATION ==============================
# Reaching the goal depends on measuring distance and heading correctly.
# DISTANCE: set CALIBRATE_STRAIGHT = True and run. The robot drives what it
#   believes is exactly 1.000 m. Measure what it really drove (D meters) and
#   set  DISTANCE_SCALE = DISTANCE_SCALE * D.  Repeat until it drives 1.00 m.
#   The calibration drives at the straight-run speed (GO_SPEED). With
#   FAST_STRAIGHT = True, adjust GO_DISTANCE_SCALE instead:
#   GO_DISTANCE_SCALE = GO_DISTANCE_SCALE * D. (DISTANCE_SCALE = 0.85 was
#   measured at normal speed and is still right for wall following.)
#   (Wheels sink into carpet and slip a little, so the distance covered per
#   wheel turn is less than the wheel's measured size suggests.)
# HEADING comes from the gyro on S1 (USE_GYRO = True). Its direction is
#   detected by the program during the first turn; if the gyro is missing or
#   dead the program falls back to the wheel encoders. START THE PROGRAM WITH
#   THE ROBOT ALREADY IN PLACE AND STILL: it calibrates the gyro, then shows
#   "READY". Press CENTER only after that.
# Encoder fallback only: CALIBRATE_SPIN = True spins what the robot believes
#   is 360 deg right; if it really turned A deg, set
#   WHEEL_BASE_M = WHEEL_BASE_M * 360 / A.
CALIBRATE_STRAIGHT = False
CALIBRATE_SPIN = False
USE_GYRO = True
DISTANCE_SCALE = 0.85       # real distance / distance from wheel size alone
GO_DISTANCE_SCALE = 1.0     # extra correction used only while driving
                            # straight at the goal (wheels slip more at
                            # FAST_STRAIGHT speed). Leave DISTANCE_SCALE
                            # alone and calibrate THIS with FAST_STRAIGHT on.
                            # (measured: drove 0.85 m when it believed 1.00 m)

# ====================== MEASURE THESE ON YOUR ROBOT ======================
WHEEL_RADIUS_M = 0.03175    # wheel radius
WHEEL_BASE_M = 0.127        # distance between the two wheel contact patches
SONAR_AHEAD_M = -0.06       # sonar position along the robot, measured from
                            # the wheel axle: + = in front, - = BEHIND it
SONAR_SIDE_M = 0.075        # robot centerline to the sonar's front face
MEASURE_AHEAD_M = 0.10      # wheel axle to measuring point, along the
                            # direction of travel (measured: 10 cm)
MEASURE_SIDE_M = 0.0        # measuring point's distance LEFT of the
                            # robot's centreline (negative = right). 0 if
                            # it is in the middle of the front.

# ================================= SPEED =================================
# Two switches. Each speed below is wheel degrees per second: (normal, fast).
#   FAST_STRAIGHT: driving straight at the goal. The first try (600) missed
#     by 26-34 cm, mostly SHORT, because the distance calibration had been
#     measured at normal speed. Now 450, with a gentle speed-up, and its
#     own calibration: GO_DISTANCE_SCALE below. CALIBRATE IT FIRST.
#   FAST_OBSTACLES: wall following, corners, hits. Fast made the position
#     estimate worse on the robot (37 cm miss vs 15 cm), so it is off.
FAST_STRAIGHT = True        # faster straight runs (see GO_DISTANCE_SCALE)
FAST_OBSTACLES = False


def pick(normal, fast, on):
    return fast if on else normal


GO_SPEED = pick(260.0, 450.0, FAST_STRAIGHT)         # straight at the goal
GOAL_SLOWDOWN = pick(900.0, 1200.0, FAST_STRAIGHT)   # final approach: wheel
                                       # deg/s per meter left (higher = full
                                       # speed until closer to the goal)
FOLLOW_SPEED = pick(260.0, 300.0, FAST_OBSTACLES)      # wall following
ARC_SPEED = pick(210.0, 300.0, FAST_OBSTACLES)         # probes at corners
CORNER_TURN_SPEED = pick(160.0, 230.0, FAST_OBSTACLES) # pivots at corners
TURN_SPEED = pick(200.0, 320.0, FAST_OBSTACLES)        # pivots after a hit
                                                       # and to face the goal
REVERSE_SPEED = pick(150.0, 250.0, FAST_OBSTACLES)     # backing up after a hit
BACKUP_DIST_M = pick(0.08, 0.06, FAST_OBSTACLES)       # back-up distance
ACCEL = 500.0               # straight runs speed up by at most this many
                            # wheel deg/s each second (less wheel slip)

# ============================ WALL FOLLOWING =============================
TARGET_DIST_M = 0.09        # desired sonar reading while following
LOST_DIST_M = 0.30          # reading above this = wall not visible
TOO_CLOSE_M = 0.06          # below this always steer away
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
CORNER_MAX_TURN_DEG = 300.0 # give up if nearly a full circle finds no wall

# ============================ OTHER MOTIONS ==============================
BUMP_TURN_DEG = 90.0        # right turn after a head-on bumper hit
SEEK_EXTRA_DEG = 15.0       # after a hit on the way to the goal: stop turning
SEEK_MIN_DEG = 20.0         # this far after the sonar first sees the wall
STALL_BACKUP_M = 0.12       # backup after a stall (side scraping a wall)
SCRAPE_ANGLE_DEG = 20.0     # assumed angle into the wall after a scrape
SCRAPE_WALL_M = 0.09        # assumed axle-to-wall distance after a scrape
SCRAPE_GENTLE_M = 0.10      # steer only gently for this far after a scrape,
SCRAPE_RADIUS_M = 0.40      # so the tail does not swing into the wall
TURN_JAM_MS = 800           # no rotation for this long = pivot is jammed
UNJAM_TURN_DEG = 10.0       # jammed pivot: turn back this far, then
UNJAM_FORWARD_M = 0.10      # drive straight this far to get clear
GLANCE_TURN_DEG = 35.0      # smaller right turn after scraping the side wall
GLANCE_DIST_M = 0.075       # sonar closer than this at a bump = scraping
STUCK_MS = 1200             # no movement for this long counts as a bump
YAW_HIT_DEG = 12.0          # driving at the goal: gyro and wheels disagree
                            # about turning by this much = the robot is
                            # being pushed round by an obstacle (0 = off)
YAW_HIT_FOLLOW_DEG = 20.0   # same check while wall following (0 = off)
TRIM = 1.5                  # small right-wheel speed correction

# ============================ BUG ALGORITHM ==============================
GOAL_TOL_M = 0.01           # stop when the measuring point is this close
GOAL_BUMP_OK_M = 0.20       # bumped this close to the goal: stop there
GOAL_FOLLOW_OK_M = 0.03     # wall following passes this close: stop there
FINAL_RADIUS_M = 0.35       # wall following passes the goal this close:
                            # turn, drive straight to it, and stop there
                            # even if this wall is in the way
LEAVE_MARGIN_M = 0.05       # must be this much closer than at the hit point
LEAVE_MIN_DEG = 5.0         # goal must be between these angles to the
LEAVE_MAX_DEG = 175.0       # robot's RIGHT before it leaves the wall
MIN_FOLLOW_M = 0.15         # follow at least this far before leaving
FULL_LOOP_DEG = 370.0       # turned this far left on one obstacle = went
                            # all the way round without being able to leave

TIME_LIMIT_MS = 170000      # stop by itself before the 3-minute limit
RESULT_HOLD_MS = 600000     # keep the result on the screen this long
                            # (10 minutes) or until CENTER is pressed

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
        self.touch_left = TouchSensor(Port.S2)    # left front bumper
        self.touch_right = TouchSensor(Port.S3)   # right front bumper
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
        self.yaw_gap = 0.0      # recent gyro turn minus wheel-encoder turn
        self.clock = StopWatch()

        # Pose in lab coordinates: (x, y) of the middle of the wheel axle.
        # heading_cw is continuous (not wrapped), in degrees, positive when
        # the robot has turned RIGHT since the start.
        # The measuring point starts on the start point, so the axle is
        # MEASURE_AHEAD_M behind it (and MEASURE_SIDE_M to its right).
        start = math.radians(START_THETA_DEG)
        self.x = (START_X_M - MEASURE_AHEAD_M * math.cos(start)
                  + MEASURE_SIDE_M * math.sin(start))
        self.y = (START_Y_M - MEASURE_AHEAD_M * math.sin(start)
                  - MEASURE_SIDE_M * math.cos(start))
        self.heading_cw = 0.0
        self.d_center = 0.0     # forward travel during the last loop
        self.d_heading = 0.0    # heading change during the last loop
        self.prev_left = 0
        self.prev_right = 0

        self.state = "GO"
        self.calibrating = False
        self.bump_x = 0.0
        self.bump_y = 0.0
        self.state_start_ms = 0

        self.turn_target = 0.0
        self.turn_start_heading = 0.0
        self.turn_seek = False        # stop the turn when the sonar sees wall
        self.turn_check_ms = 0
        self.turn_check_heading = 0.0
        self.bump_turn = BUMP_TURN_DEG
        self.after_turn = "FOLLOW"
        self.backup_dist = BACKUP_DIST_M
        self.backup_stall = False     # backing up after a stall, not a bump
        self.backup_seek = False
        self.scrape = False           # start following with a wall touching
        self.scrape_left = 0.0        # gentle-steering distance still to go
        self.unjam_turning = True
        self.unjam_dir = 1.0
        self.unjam_heading = 0.0
        self.unjam_x = 0.0
        self.unjam_y = 0.0

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

        # Bug algorithm
        self.hit_d = 99.0             # distance to the goal at the hit point
        self.new_hit = False
        self.hits = 0                 # obstacles met on the way to the goal
        self.bumps = 0                # other bumps and stalls while following
        self.episode_travel = 0.0     # wall following since the hit point
        self.episode_heading = 0.0    # heading when this obstacle was met
        self.episode_min_tip = 99.0   # closest the tip got to the goal
        self.go_min_tip = 99.0
        self.final = False            # on the last short drive to the goal

        self.stuck_start_ms = 0
        self.stuck_travel = 0.0
        self.debug_counter = 0
        self.outcome = "gave up"      # becomes "goal", "closest", "time up"

    # ------------------------------ helpers ------------------------------

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def halt(self):
        """Stop dead (brake) instead of coasting: used for the final stop."""
        self.left_motor.brake()
        self.right_motor.brake()

    def steer(self, speed, turn):
        # Positive turn curves RIGHT; negative turn curves LEFT.
        self.left_motor.run(speed + turn)
        self.right_motor.run(speed - turn + TRIM)

    def pivot(self, speed, right):
        if right:
            self.left_motor.run(speed)
            self.right_motor.run(-speed)
        else:
            self.left_motor.run(-speed)
            self.right_motor.run(speed)

    def finale(self):
        """Stop, beep, and show where the robot thinks it is.

        The numbers stay on the screen until CENTER is pressed (or for
        RESULT_HOLD_MS), so they can be written down and compared with a
        tape measure. All positions are in meters in the lab's coordinates.
        """
        seconds = self.clock.time() / 1000.0
        self.ev3.speaker.beep()
        if self.outcome == "goal":
            self.ev3.light.on(Color.GREEN)
            tune = VICTORY_TUNE
        else:
            self.ev3.light.on(Color.RED)
            tune = SAD_TUNE
        tip_x, tip_y = self.tip_pos()
        facing = wrap_degrees(math.degrees(self.theta()))
        # Five lines only: more than that scrolls off the EV3 screen.
        lines = [
            "{} {:.0f}s".format(self.outcome.upper(), seconds),
            "TIP x{:.2f} y{:.2f}".format(tip_x, tip_y),
            "axl x{:.2f} y{:.2f}".format(self.x, self.y),
            "facing {:.0f}".format(facing),
            "hits {} bumps {}".format(self.hits, self.bumps),
        ]
        self.ev3.screen.clear()
        for line in lines:
            self.ev3.screen.print(line)
            print(line)
        for frequency, duration in tune:
            if frequency > 0:
                self.ev3.speaker.beep(frequency, duration)
            else:
                wait(duration)
            wait(30)
        # Keep the result on the screen.
        hold = StopWatch()
        while (hold.time() < RESULT_HOLD_MS
               and Button.CENTER not in self.ev3.buttons.pressed()):
            wait(50)

    def say(self, message):
        self.ev3.screen.clear()
        self.ev3.screen.print(message)
        print(message)

    def theta(self):
        """Direction the robot faces, radians counter-clockwise from +x."""
        return math.radians(START_THETA_DEG - self.heading_cw)

    def update_odometry(self):
        l = self.left_motor.angle()
        r = self.right_motor.angle()
        d_l = (math.radians(l - self.prev_left) * WHEEL_RADIUS_M
               * DISTANCE_SCALE)
        d_r = (math.radians(r - self.prev_right) * WHEEL_RADIUS_M
               * DISTANCE_SCALE)
        self.prev_left = l
        self.prev_right = r
        self.d_center = (d_l + d_r) / 2.0
        if self.state == "GO" or self.calibrating:
            self.d_center *= GO_DISTANCE_SCALE
        enc_d = math.degrees((d_l - d_r) / WHEEL_BASE_M)
        self.enc_turn += enc_d
        if self.gyro_sign != 0:
            raw = self.gyro.angle()
            self.d_heading = self.gyro_sign * (raw - self.gyro_prev)
            self.gyro_prev = raw
            # Leaky sum (about the last second) of how much more the robot
            # really turned than its wheels say it should have.
            self.yaw_gap = 0.97 * self.yaw_gap + (self.d_heading - enc_d)
        else:
            self.d_heading = enc_d
        mid = math.radians(START_THETA_DEG - self.heading_cw
                           - self.d_heading / 2.0)
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

    def tip_pos(self):
        """Where the measuring point is, from the axle position and heading."""
        a = self.theta()
        return (self.x + MEASURE_AHEAD_M * math.cos(a)
                - MEASURE_SIDE_M * math.sin(a),
                self.y + MEASURE_AHEAD_M * math.sin(a)
                + MEASURE_SIDE_M * math.cos(a))

    def tip_dist_to_goal(self):
        """Distance from the measuring point to the goal."""
        tip_x, tip_y = self.tip_pos()
        dx = tip_x - AIM_X_M
        dy = tip_y - AIM_Y_M
        return math.sqrt(dx * dx + dy * dy)

    def bearing_error_to(self, px, py):
        """Degrees the robot must turn (right positive) to face (px, py)."""
        desired_cw = START_THETA_DEG - math.degrees(
            math.atan2(py - self.y, px - self.x))
        return wrap_degrees(desired_cw - self.heading_cw)

    def set_state(self, state):
        self.state = state
        self.state_start_ms = self.clock.time()
        self.stuck_start_ms = self.state_start_ms
        self.stuck_travel = 0.0
        self.yaw_gap = 0.0

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

    def begin_backup(self, turn_degrees, next_state, seek=False, stall=False):
        """Reverse a little, then turn right and go to next_state.

        seek:  end the turn early once the sonar sees the wall on the left.
        stall: the robot stopped without the bumper being pressed.
        """
        self.stop()
        self.bump_turn = turn_degrees
        self.after_turn = next_state
        self.backup_seek = seek
        self.backup_stall = stall
        self.backup_dist = STALL_BACKUP_M if stall else BACKUP_DIST_M
        self.bump_x, self.bump_y = self.x, self.y
        self.set_state("BACKUP")

    def begin_turn(self, turn_degrees, next_state, seek=False):
        """Pivot in place by turn_degrees (right positive), then next_state."""
        self.stop()
        self.turn_target = self.heading_cw + turn_degrees
        self.turn_start_heading = self.heading_cw
        self.turn_seek = seek
        self.turn_check_ms = self.clock.time()
        self.turn_check_heading = self.heading_cw
        self.after_turn = next_state
        self.enc_turn0 = self.enc_turn
        if self.gyro is not None:
            self.gyro_turn0 = self.gyro.angle()
        self.set_state("TURN")

    def begin_go(self):
        """Face the goal, then drive at it."""
        self.go_min_tip = 99.0
        err = self.bearing_error_to(AIM_X_M, AIM_Y_M)
        if abs(err) > 8.0:
            self.begin_turn(err, "GO")
        else:
            self.set_state("GO")
        self.say("GO")

    def begin_follow(self):
        if self.new_hit:
            # First wall of a new obstacle.
            self.new_hit = False
            self.episode_travel = 0.0
            self.episode_heading = self.heading_cw
            self.episode_min_tip = 99.0
        self.mode = "TRACK"
        self.track_dist = 0.0
        self.lost_travel = 0.0
        for _ in range(3):
            wall = self.read_wall()
            wait(15)
        if self.scrape:
            # The robot's left side was touching the wall, nose angled in.
            # Start from that, so it steers away even if the sonar is too
            # close to the wall to get a reading.
            self.scrape = False
            self.scrape_left = SCRAPE_GENTLE_M
            self.reset_wall_estimate(wall, 0.0)
            self.wall_phi = math.radians(SCRAPE_ANGLE_DEG)
            self.wall_d = min(self.wall_d, SCRAPE_WALL_M)
        else:
            # Start with the corner search's straight probe: if the sonar
            # sees the wall it measures the wall's angle over a few cm and
            # then tracks it; if not, it pivots left to find the wall.
            self.reset_wall_estimate(wall, 0.0)
            a = self.theta()
            self.corner_x = (self.x + SONAR_AHEAD_M * math.cos(a)
                             - AXLE_TARGET_M * math.sin(a))
            self.corner_y = (self.y + SONAR_AHEAD_M * math.sin(a)
                             + AXLE_TARGET_M * math.cos(a))
            self.corner_start_heading = self.heading_cw
            self.seen_travel = 0.0
            self.mode = "CORNER"
        self.corner_pivoting = False
        self.set_state("FOLLOW")
        self.say("FOLLOW")

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

        Prediction uses the measured motion: a left turn increases the angle,
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
        if (wall is not None and wall < TOO_CLOSE_M
                and self.wall_phi > -0.5 * limit):
            # Too close and not already angled away: steer away. (When the
            # nose already points away, a harder right turn would only
            # swing the tail into the wall.)
            turn = max(turn, 0.3 * FOLLOW_SPEED)
        if self.scrape_left > 0.0:
            # Just freed from a scrape or a jammed turn: a wide arc only.
            self.scrape_left -= abs(self.d_center)
            gentle = FOLLOW_SPEED * WHEEL_BASE_M / (2.0 * SCRAPE_RADIUS_M)
            turn = clamp(turn, -gentle, gentle)
        self.steer(FOLLOW_SPEED, turn)

    def start_corner(self):
        """The wall just ended. Work out where its end (the corner) is."""
        a = self.theta()
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
                self.pivot(CORNER_TURN_SPEED, False)
            self.yaw_gap = 0.0            # wheels slip in pivots: not a hit
            return

        # Straight probe. How far is the SONAR past the corner, measured
        # along the direction the robot is now facing? Until it is past,
        # the next wall cannot be in front of the sonar, and anything it
        # hears is the old wall or the corner edge.
        a = self.theta()
        past = ((self.x - self.corner_x) * math.cos(a)
                + (self.y - self.corner_y) * math.sin(a) + SONAR_AHEAD_M)
        if seen:
            if self.seen_travel == 0.0:
                self.seen_first = wall
            self.seen_travel += step
        else:
            self.seen_travel = 0.0
        confirmed = self.seen_travel >= CORNER_CONFIRM_M

        if confirmed and turned < 5.0 and self.track_dist > 0.02:
            self.mode = "TRACK"           # same wall: it was only a dropout
            self.lost_travel = 0.0
            self.track(wall)
        elif confirmed and (past >= CORNER_PAST_M or turned < 5.0):
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
                self.begin_go()           # nothing to follow: head for goal
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

    # ------------------------------ start-up -----------------------------

    def calibrate_gyro(self):
        """Switching the gyro between its two modes makes it re-zero itself.
        The robot must be still while this happens."""
        try:
            self.gyro.speed()
            wait(300)
            self.gyro.angle()
            wait(700)
            self.gyro.reset_angle(0)
        except Exception:
            self.gyro = None
            print("GYRO NOT RESPONDING - using encoders")

    def wait_for_start(self):
        """Wait for CENTER. Meanwhile check that the gyro is not drifting."""
        self.stop()
        if self.gyro is not None:
            self.say("Hold still...")
            self.calibrate_gyro()
        self.say("READY: press CENTER")
        check_ms = self.clock.time()
        check_angle = self.gyro.angle() if self.gyro is not None else 0
        while Button.CENTER not in self.ev3.buttons.pressed():
            wait(20)
            if (self.gyro is not None
                    and self.clock.time() - check_ms >= 2000):
                if abs(self.gyro.angle() - check_angle) >= 2:
                    # Moved or drifting: calibrate again.
                    self.say("GYRO DRIFT: hold still")
                    self.calibrate_gyro()
                    self.say("READY: press CENTER")
                check_ms = self.clock.time()
                check_angle = (self.gyro.angle()
                               if self.gyro is not None else 0)
        self.clock.reset()                # the race clock starts here
        wait(250)                         # let the finger come off
        self.prev_left = self.left_motor.angle()
        self.prev_right = self.right_motor.angle()
        if self.gyro is not None:
            self.gyro.reset_angle(0)

    def calibration_run(self):
        if CALIBRATE_STRAIGHT:
            # Drive what the robot believes is exactly 1.000 m.
            # Same speed-up and slow-down as a real straight run, then
            # brake, so coasting does not add to the measured distance.
            self.calibrating = True
            x0, y0 = self.x, self.y
            while (self.dist_to(x0, y0) < 1.0
                   and self.clock.time() < 20000):
                left = 1.0 - self.dist_to(x0, y0)
                speed = clamp(left * GOAL_SLOWDOWN, 50.0, GO_SPEED)
                speed = min(speed, 100.0 + ACCEL * self.clock.time() / 1000.0)
                turn = clamp(-3.0 * self.enc_turn, -40, 40)
                self.steer(speed, turn)
                wait(LOOP_MS)
                self.update_odometry()
            self.halt()
            self.calibrating = False
            self.say("DROVE 1.00 m?")
        else:
            # Spin what the robot believes is exactly 360 degrees right.
            while self.enc_turn < 360.0 and self.clock.time() < 20000:
                self.pivot(clamp((360.0 - self.enc_turn) * 5.0, 60.0, 150.0),
                           True)
                wait(LOOP_MS)
                self.update_odometry()
            self.stop()
            self.say("SPUN 360?")
        wait(5000)

    # ------------------------------ main loop ----------------------------

    def run(self):
        self.wait_for_start()
        if CALIBRATE_STRAIGHT or CALIBRATE_SPIN:
            self.calibration_run()
            return

        self.begin_go()

        while True:
            wait(LOOP_MS)
            now = self.clock.time()
            if now > TIME_LIMIT_MS:
                self.say("TIME UP")
                self.outcome = "time up"
                break

            self.update_odometry()
            wall = self.read_wall()
            hit_left = self.touch_left.pressed()
            hit_right = self.touch_right.pressed()
            bumped = hit_left or hit_right
            goal_d = self.dist_to(AIM_X_M, AIM_Y_M)      # from the axle
            tip_d = self.tip_dist_to_goal()                # from the tip
            err = self.bearing_error_to(AIM_X_M, AIM_Y_M)

            if self.state == "GO":
                # The bumper is narrow: an angled wall can stop the robot
                # without pressing it, so a stall also counts as a hit.
                stuck = self.is_stuck(now)
                if (YAW_HIT_DEG > 0 and abs(self.yaw_gap) >= YAW_HIT_DEG
                        and now - self.state_start_ms > 500):
                    # Wheels say "straight" but the gyro says the robot is
                    # being turned: one side is caught on an obstacle.
                    if DEBUG:
                        print("YAW HIT gap={:.0f}".format(self.yaw_gap))
                    stuck = True
                if tip_d < self.go_min_tip:
                    self.go_min_tip = tip_d
                passed = (self.go_min_tip < 0.20
                          and tip_d > self.go_min_tip + 0.006)
                if bumped or stuck:
                    if tip_d <= GOAL_BUMP_OK_M or self.final:
                        self.outcome = "goal"
                        break
                    # Hit point: remember how far the goal was from here.
                    self.hits += 1
                    self.hit_d = goal_d
                    self.new_hit = True
                    print("HIT {} at x={:.2f} y={:.2f} goal={:.2f}".format(
                        self.hits, self.x, self.y, goal_d))
                    self.begin_backup(BUMP_TURN_DEG, "FOLLOW",
                                      seek=bumped, stall=not bumped)
                elif tip_d <= GOAL_TOL_M or passed:
                    self.outcome = "goal"
                    break
                elif abs(err) > 35.0 and goal_d > 0.10:
                    self.begin_turn(err, "GO")     # aim again
                    self.go_min_tip = 99.0
                else:
                    # Full speed until close, then slow to a crawl so the
                    # stop is exact.
                    speed = clamp(tip_d * GOAL_SLOWDOWN, 50.0, GO_SPEED)
                    # Gentle speed-up: sudden starts make the wheels slip.
                    speed = min(speed, 100.0 + ACCEL
                                * (now - self.state_start_ms) / 1000.0)
                    turn = clamp(2.5 * err, -0.4 * speed, 0.4 * speed)
                    self.steer(speed, turn)

            elif self.state == "BACKUP":
                self.left_motor.run(-REVERSE_SPEED)
                self.right_motor.run(-REVERSE_SPEED)
                moved = self.dist_to(self.bump_x, self.bump_y)
                if (moved >= self.backup_dist or self.is_stuck(now)
                        or now - self.state_start_ms > 3000):
                    if moved < 0.03:
                        # Cannot reverse: the tail is against something.
                        # Turn the nose left a little and drive clear.
                        self.stop()
                        if DEBUG:
                            print("BACKUP BLOCKED")
                        self.unjam_turning = True
                        self.unjam_dir = -1.0
                        self.unjam_heading = self.heading_cw
                        self.set_state("UNJAM")
                    elif self.backup_stall and wall <= LOST_DIST_M:
                        # Scraping a wall that is on the left: no turn.
                        self.stop()
                        self.scrape = True
                        self.begin_follow()
                    else:
                        self.begin_turn(self.bump_turn, self.after_turn,
                                        self.backup_seek)

            elif self.state == "TURN":
                if self.gyro is not None and self.gyro_sign == 0:
                    # Compare gyro and encoders over the first real turn to
                    # learn which way the gyro counts, then switch to it.
                    enc = self.enc_turn - self.enc_turn0
                    if abs(enc) >= 25.0:
                        raw = self.gyro.angle()
                        gyro = raw - self.gyro_turn0
                        if abs(gyro) >= 10.0:
                            self.gyro_sign = 1 if gyro * enc > 0 else -1
                            self.gyro_prev = raw
                            self.heading_cw += self.gyro_sign * gyro - enc
                            print("GYRO OK sign={} gyro={} enc={:.0f}".format(
                                self.gyro_sign, gyro, enc))
                        else:
                            self.gyro = None
                            print("GYRO NOT RESPONDING - using encoders")
                if self.turn_seek and wall <= LOST_DIST_M:
                    # The sonar sees the wall, so the robot is close to
                    # parallel with it: turn only a little further.
                    turned = self.heading_cw - self.turn_start_heading
                    self.turn_target = min(
                        self.turn_target, self.turn_start_heading
                        + max(SEEK_MIN_DEG, turned + SEEK_EXTRA_DEG))
                    self.turn_seek = False
                # Jammed (for example the tail against a wall)?
                jammed = False
                if now - self.turn_check_ms >= TURN_JAM_MS:
                    jammed = abs(self.heading_cw
                                 - self.turn_check_heading) < 3.0
                    self.turn_check_ms = now
                    self.turn_check_heading = self.heading_cw
                remaining = self.turn_target - self.heading_cw
                if jammed:
                    # Something stopped the pivot (usually the tail against
                    # a wall). Turn back a little, then drive clear.
                    self.stop()
                    if DEBUG:
                        print("TURN JAMMED")
                    self.unjam_turning = True
                    self.unjam_dir = -1.0 if remaining > 0 else 1.0
                    self.unjam_heading = self.heading_cw
                    self.set_state("UNJAM")
                elif (abs(remaining) <= 2.0
                        or now - self.state_start_ms > 6000):
                    self.stop()
                    wait(100)
                    if self.after_turn == "FOLLOW":
                        self.begin_follow()
                    else:
                        self.go_min_tip = 99.0
                        self.set_state("GO")
                else:
                    self.pivot(clamp(abs(remaining) * 5.0, 70.0, TURN_SPEED),
                               remaining > 0)

            elif self.state == "UNJAM":
                if self.unjam_turning:
                    turned_back = abs(self.heading_cw - self.unjam_heading)
                    if (turned_back >= UNJAM_TURN_DEG
                            or now - self.state_start_ms > 1000):
                        self.stop()
                        self.unjam_turning = False
                        self.unjam_x, self.unjam_y = self.x, self.y
                        self.set_state("UNJAM")       # restart its timers
                    else:
                        self.pivot(CORNER_TURN_SPEED, self.unjam_dir > 0)
                else:
                    stuck = self.is_stuck(now)
                    if bumped or stuck:
                        self.begin_backup(BUMP_TURN_DEG, self.after_turn,
                                          stall=not bumped)
                    elif (self.dist_to(self.unjam_x, self.unjam_y)
                            >= UNJAM_FORWARD_M):
                        if self.after_turn == "FOLLOW":
                            self.begin_follow()
                        else:
                            self.go_min_tip = 99.0
                            self.set_state("GO")
                    else:
                        self.steer(ARC_SPEED, 0.0)

            elif self.state == "FOLLOW":
                self.episode_travel += abs(self.d_center)
                if tip_d < self.episode_min_tip:
                    self.episode_min_tip = tip_d

                if tip_d <= GOAL_FOLLOW_OK_M:
                    self.outcome = "goal"
                    break

                if (tip_d <= FINAL_RADIUS_M
                        and tip_d > self.episode_min_tip + 0.01
                        and not self.corner_pivoting):
                    # Just passed the goal while following: go straight to it.
                    print("FINAL at x={:.2f} y={:.2f} tip={:.2f}".format(
                        self.x, self.y, tip_d))
                    self.final = True
                    self.begin_go()
                    continue

                stuck = self.is_stuck(now)
                if (YAW_HIT_FOLLOW_DEG > 0
                        and abs(self.yaw_gap) >= YAW_HIT_FOLLOW_DEG
                        and now - self.state_start_ms > 500):
                    if DEBUG:
                        print("YAW HIT gap={:.0f}".format(self.yaw_gap))
                    stuck = True
                if bumped or stuck:
                    # Scraping the wall we are following (it is on the
                    # LEFT, so only the left bumper is pressed, or the sonar
                    # reads very close) needs only a small turn away; a wall
                    # ahead (right bumper pressed) needs the full 90.
                    glancing = self.mode == "TRACK" and (
                        (hit_left and not hit_right)
                        or (not bumped and wall <= GLANCE_DIST_M))
                    self.bumps += 1
                    if DEBUG:
                        print("BUMP L={} R={} wall={:.2f} glancing={}".format(
                            hit_left, hit_right, wall, glancing))
                    self.begin_backup(GLANCE_TURN_DEG if glancing
                                      else BUMP_TURN_DEG, "FOLLOW",
                                      stall=not bumped)
                    continue

                # LEAVE: closer to the goal than at the hit point, and the
                # goal is off to the right, away from the wall on the left.
                if (self.episode_travel >= MIN_FOLLOW_M
                        and goal_d < self.hit_d - LEAVE_MARGIN_M
                        and LEAVE_MIN_DEG <= err <= LEAVE_MAX_DEG
                        and not self.corner_pivoting):
                    print("LEAVE at x={:.2f} y={:.2f} goal={:.2f}".format(
                        self.x, self.y, goal_d))
                    self.begin_go()
                    continue

                # Safety net: all the way round this obstacle and never able
                # to leave. Stop at the closest point to the goal it found.
                if (self.episode_heading - self.heading_cw >= FULL_LOOP_DEG
                        and tip_d <= self.episode_min_tip + 0.04):
                    self.outcome = "closest"
                    break

                self.follow_step(wall)
                if self.state != "FOLLOW":
                    continue

                self.debug_counter += 1
                if DEBUG and self.debug_counter % 25 == 0:
                    print("{} t={:.0f} x={:.2f} y={:.2f} goal={:.2f} "
                          "err={:.0f} wall={:.2f}"
                          .format(self.mode, now / 1000.0, self.x, self.y,
                                  goal_d, err, wall))

        self.halt()
        print("STOPPED {} t={:.1f} x={:.2f} y={:.2f} tip_to_goal={:.2f}".format(
            self.outcome, self.clock.time() / 1000.0, self.x, self.y,
            self.tip_dist_to_goal()))
        self.finale()


def main():
    robot = Robot()
    try:
        robot.run()
    finally:
        robot.stop()


if __name__ == "__main__":
    main()