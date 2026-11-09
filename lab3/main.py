#!/usr/bin/env pybricks-micropython
# COMP 581 Lab 3 - Team #__
# Names / PIDs: ______________________

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch
import math

# ----------------------------------------------------------------------
# ASSUMPTIONS:
#   * Ultrasonic sensor faces LEFT (wall is on the left after the right turn).
#   * Positive steer = curve RIGHT (left wheel faster).
#   * Robot frame: start = (0,0), initial heading = +x. theta is CCW-positive
#     (the EV3 gyro is CW-positive, so theta = -gyro).
#
# CORNER HANDLING (outside/convex corner, wall lost) - works for any angle:
#   arc left FAST until the sensor sees a wall again (or MAX_ARC degrees),
#   hug it with a strong raw-distance controller for HUG_DIST meters,
#   then resume normal gyro-corrected wall following.
# ----------------------------------------------------------------------


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class MyController:
    def __init__(self):
        self.ev3 = EV3Brick()
        self.left_motor = Motor(Port.C)
        self.right_motor = Motor(Port.D)
        self.touch_sensor1 = TouchSensor(Port.S3)
        self.ultrasonic_sensor = UltrasonicSensor(Port.S4)
        self.gyro_sensor = GyroSensor(Port.S1)
        self.watch = StopWatch()
        wait(1000)  # let the gyro settle; do not move the robot

        # Geometry (meters)
        self.WHEEL_RADIUS = 0.03175
        self.WHEEL_BASE = 0.127
        # Distance from robot center to the measuring point (front of robot).
        self.MEASURE_OFFSET = 0.0

        # Speeds (deg/s of the wheel motors)
        self.APPROACH_SPEED = 200
        self.REVERSE_SPEED = 120
        self.TURN_SPEED = 200
        self.FOLLOW_SPEED = 140
        self.RETURN_SPEED = 180

        # Wall following (P control)
        self.TARGET_DIST = 0.15     # want ~15 cm from wall (limit is 30 cm)
        self.KP_WALL = 250.0        # deg/s of steer per meter of error
        self.MAX_STEER = 45
        self.LOST_DIST = 0.32       # wall considered lost beyond this
        self.MAX_US = 0.60          # clamp ultrasonic readings

        # Corner handling (tune these!)
        self.LOST_CONFIRM = 0.02    # meters of travel with no wall before a corner is declared
        self.CORNER_SPEED = 120     # avg wheel speed during the arc
        self.ARC_STEER = 75         # arc sharpness: left wheel 45, right 195 (radius ~10 cm, ~37 deg/s)
        self.ARC_EASE_DEG = 45      # after this many degrees of arc, halve the steering
        self.SEE_DIST = 0.28        # arc ends when the sensor reads closer than this...
        self.SEE_LOOPS = 4          # ...for this many loops in a row...
        self.MIN_ARC = 8            # ...after turning at least this many degrees
        self.MAX_ARC = 100          # give up arcing after this many degrees
        self.MIN_TRACK = 0.15       # must follow a wall this far (m) before a loss counts as a corner
        self.track_dist = 0.0
        self.KP_HUG = 400.0         # stronger P control while re-hugging after a corner
        self.MAX_STEER_HUG = 90
        self.HUG_DIST = 0.20        # meters of "hug" driving before normal following resumes
        self.has_wall = False       # seen the wall since the last turn?
        # 0 = following, 2 = arcing left, 3 = hugging after corner, 4 = searching straight
        self.lost_phase = 0
        self.lost_count = 0
        self.see_count = 0
        self.hug_travel = 0.0
        self.lost_gyro = 0.0

        # Bump recovery
        self.BACKUP_DIST = 0.10

        # Loop closure
        self.MIN_LOOP_DIST = 1.2    # must follow at least this far (min perimeter 1.5 m)
        self.HIT_RADIUS = 0.30      # "near the hit point"

        # Heading hold while driving straight
        self.KP_HEADING = 3.0

        # Hardware drift trim (deg/s added to right wheel). Original code used 1.5.
        # If the robot drifts LEFT when driving straight, raise it; RIGHT, lower it.
        self.TRIM = 1.5

        self.dbg = 0
        self.KP_YAW = 2.0          # deg/s of steer per degree of yaw toward the wall
        self.wall_ref = 0.0        # gyro heading parallel to the wall
        self.TIME_LIMIT_MS = 118000

        # State
        self.state = "WAITING"
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.d_center = 0.0
        self.prev_left = self.left_motor.angle()
        self.prev_right = self.right_motor.angle()

        self.start_x = self.MEASURE_OFFSET
        self.start_y = 0.0
        self.hit_x = 0.0
        self.hit_y = 0.0
        self.hit_saved = False
        self.bump_x = 0.0
        self.bump_y = 0.0

        self.follow_dist = 0.0
        self.turn_target = 0.0
        self.after_turn = "FOLLOW"
        self.wall_f = 0.3  # filtered wall distance

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def steer(self, speed, turn):
        # positive turn = curve right
        self.left_motor.run(speed + turn)
        self.right_motor.run(speed - turn + self.TRIM)

    def say(self, msg):
        self.ev3.screen.clear()
        self.ev3.screen.print(msg)
        print(msg)

    def update_odometry(self):
        l = self.left_motor.angle()
        r = self.right_motor.angle()
        d_left = math.radians(l - self.prev_left) * self.WHEEL_RADIUS
        d_right = math.radians(r - self.prev_right) * self.WHEEL_RADIUS
        self.prev_left = l
        self.prev_right = r

        self.d_center = (d_left + d_right) / 2.0
        # Gyro is clockwise-positive, math is CCW-positive -> negate.
        self.theta = -math.radians(self.gyro_sensor.angle())
        self.x += self.d_center * math.cos(self.theta)
        self.y += self.d_center * math.sin(self.theta)

    def read_wall(self):
        raw = self.ultrasonic_sensor.distance() / 1000.0
        raw = min(raw, self.MAX_US)
        self.wall_f = 0.5 * self.wall_f + 0.5 * raw
        return self.wall_f

    def dist_to(self, px, py):
        return math.sqrt((self.x - px) ** 2 + (self.y - py) ** 2)

    def begin_turn_right(self, degrees, next_state):
        self.turn_target = self.gyro_sensor.angle() + degrees
        self.after_turn = next_state
        self.state = "TURN"

    def wall_follow(self, wall):
        # Wall is on the left.
        g = self.gyro_sensor.angle()
        # How far we are yawed LEFT (toward the wall) relative to the wall's
        # direction. Gyro is CW-positive, so yawing left makes g smaller.
        yaw_left = clamp(math.radians(self.wall_ref - g), -0.7, 0.7)

        # A sideways sensor reads along the beam: when yawed, the reading is
        # longer than the true perpendicular distance. Correct it.
        perp = wall * math.cos(yaw_left)

        # Too close -> error>0 -> steer right (away from wall).
        error = self.TARGET_DIST - perp
        turn = self.KP_WALL * error + self.KP_YAW * math.degrees(yaw_left)
        turn = clamp(turn, -self.MAX_STEER, self.MAX_STEER)
        self.steer(self.FOLLOW_SPEED, turn)

        # When we're sitting at the right distance we must be roughly parallel
        # to the wall, so slowly learn the wall direction (handles curved walls).
        if abs(error) < 0.03:
            self.wall_ref += 0.03 * (g - self.wall_ref)

    def start_arc(self, g):
        self.say("ARC w={:.2f}".format(self.wall_f))
        self.lost_phase = 2
        self.lost_count = 0
        self.see_count = 0
        self.lost_gyro = g

    def start_hug(self):
        self.say("HUG w={:.2f}".format(self.wall_f))
        self.lost_phase = 3
        self.hug_travel = 0.0
        self.lost_count = 0
        self.see_count = 0

    def follow_step(self, wall):
        """One control step of wall following, including corner handling.

        Corner handling is sensor-driven (works for any corner angle):
          arc left FAST until the sensor sees a wall again, then hug it with a
          strong raw-distance controller for a short distance, then resume
          normal gyro-corrected following.
        """
        g = self.gyro_sensor.angle()

        # ---- phase 0: normal following ----
        if self.lost_phase == 0:
            if wall <= self.LOST_DIST:
                self.track_dist += abs(self.d_center)
                if self.track_dist >= self.MIN_TRACK:
                    self.has_wall = True
                self.lost_count = 0
                self.wall_follow(wall)
            elif not self.has_wall:
                # Haven't seen the wall since the turn: straight.
                self.steer(self.FOLLOW_SPEED * 0.7, 0)
            else:
                # lost_count = meters traveled with no wall (not loops): the
                # sensor refreshes slower than our loop.
                self.lost_count += abs(self.d_center)
                if self.lost_count < self.LOST_CONFIRM:
                    self.steer(self.FOLLOW_SPEED, 0)  # brief dropout
                else:
                    self.start_arc(g)

        # ---- phase 2: fast left arc until the sensor sees a wall ----
        elif self.lost_phase == 2:
            turned_left = self.lost_gyro - g  # left = gyro decreases
            if wall < self.SEE_DIST and turned_left >= self.MIN_ARC:
                self.see_count += 1
            else:
                self.see_count = 0
            if self.see_count >= self.SEE_LOOPS:
                self.start_hug()
            elif turned_left >= self.MAX_ARC:
                self.say("SEARCH w={:.2f}".format(self.wall_f))
                self.lost_phase = 4  # nothing found: go straight and search
            else:
                # Ease off partway through so we don't overshoot the new wall.
                steer_now = self.ARC_STEER
                if turned_left >= self.ARC_EASE_DEG:
                    steer_now = self.ARC_STEER * 0.5
                self.steer(self.CORNER_SPEED, -steer_now)

        # ---- phase 3: hug the corner / new wall with raw distance control ----
        elif self.lost_phase == 3:
            # Raw reading only (no yaw correction): holding a constant raw
            # distance curls the robot around a corner point. Keep wall_ref
            # tracking the heading so normal following starts from here.
            self.wall_ref = g
            self.hug_travel += abs(self.d_center)
            if wall > self.LOST_DIST:
                self.lost_count += abs(self.d_center)
                if self.lost_count >= self.LOST_CONFIRM:
                    self.start_arc(g)  # another corner right away
                else:
                    self.steer(self.CORNER_SPEED, 0)
            else:
                self.lost_count = 0
                error = self.TARGET_DIST - wall
                turn = clamp(self.KP_HUG * error, -self.MAX_STEER_HUG, self.MAX_STEER_HUG)
                self.steer(self.CORNER_SPEED, turn)
                if self.hug_travel >= self.HUG_DIST:
                    self.lost_phase = 0
                    self.lost_count = 0

        # ---- phase 4: arc found nothing; straight until a wall appears ----
        else:
            if wall <= self.LOST_DIST:
                self.start_hug()
            else:
                self.steer(self.CORNER_SPEED, -8)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        while True:
            wait(20)

            if self.state == "WAITING":
                self.stop()
                self.ev3.screen.clear()
                self.ev3.screen.print("Press CENTER")
                if Button.CENTER in self.ev3.buttons.pressed():
                    wait(500)  # let go of the button / robot settle
                    self.gyro_sensor.reset_angle(0)  # only reset here, never again
                    self.x = 0.0
                    self.y = 0.0
                    self.theta = 0.0
                    self.follow_dist = 0.0
                    self.prev_left = self.left_motor.angle()
                    self.prev_right = self.right_motor.angle()
                    self.hit_saved = False
                    self.watch.reset()
                    self.state = "DRIVE"
                continue

            if self.state == "DONE":
                self.stop()
                break

            if self.watch.time() > self.TIME_LIMIT_MS:
                self.stop()
                self.say("TIME UP")
                break

            self.update_odometry()
            wall = self.read_wall()

            # ---------------- DRIVE: straight toward the obstacle ----------
            if self.state == "DRIVE":
                heading_deg = self.gyro_sensor.angle()  # CW positive
                turn = clamp(-self.KP_HEADING * heading_deg, -40, 40)
                self.steer(self.APPROACH_SPEED, turn)

                if self.touch_sensor1.pressed():
                    self.stop()
                    self.ev3.speaker.beep()
                    self.hit_x = self.x
                    self.hit_y = self.y
                    self.hit_saved = True
                    self.bump_x = self.x
                    self.bump_y = self.y
                    self.say("HIT")
                    self.state = "BACKUP"

            # ---------------- BACKUP: reverse a fixed distance -------------
            elif self.state == "BACKUP":
                self.left_motor.run(-self.REVERSE_SPEED)
                self.right_motor.run(-self.REVERSE_SPEED)
                if self.dist_to(self.bump_x, self.bump_y) >= self.BACKUP_DIST:
                    self.stop()
                    self.begin_turn_right(90, "FOLLOW")

            # ---------------- TURN: gyro-based right turn ------------------
            elif self.state == "TURN":
                remaining = self.turn_target - self.gyro_sensor.angle()
                if remaining > 1:
                    speed = clamp(remaining * 5, 40, self.TURN_SPEED)
                    self.left_motor.run(speed)
                    self.right_motor.run(-speed)
                else:
                    self.stop()
                    wait(100)
                    self.wall_f = self.ultrasonic_sensor.distance() / 1000.0
                    self.wall_f = min(self.wall_f, self.MAX_US)
                    self.has_wall = False
                    self.track_dist = 0.0
                    self.lost_phase = 0
                    self.lost_count = 0
                    self.wall_ref = self.gyro_sensor.angle()
                    if self.after_turn == "FOLLOW":
                        self.say("FOLLOW")
                    elif self.after_turn == "RETURN":
                        self.say("RETURN")
                    self.state = self.after_turn

            # ---------------- FOLLOW: wall on the left ---------------------
            elif self.state == "FOLLOW":
                self.follow_dist += abs(self.d_center)

                # Bumped something ahead (inside corner): back up, turn right.
                if self.touch_sensor1.pressed():
                    self.stop()
                    self.bump_x = self.x
                    self.bump_y = self.y
                    self.state = "BACKUP"
                    continue

                self.dbg += 1
                if self.dbg % 25 == 0:
                    self.ev3.screen.print("{:.2f}".format(wall))
                self.follow_step(wall)

                # Loop closure: gone far enough AND back near the hit point.
                if (self.hit_saved and self.follow_dist > self.MIN_LOOP_DIST
                        and self.dist_to(self.hit_x, self.hit_y) < self.HIT_RADIUS):
                    self.stop()
                    self.ev3.speaker.beep()
                    self.say("BACK AT HIT")
                    wait(300)
                    # Turn RIGHT (away from the wall on our left) to face the start.
                    bearing = math.atan2(self.start_y - self.y, self.start_x - self.x)
                    rot = (self.theta - bearing) % (2 * math.pi)  # CW amount
                    self.begin_turn_right(math.degrees(rot), "RETURN")

            # ---------------- RETURN: drive to the start -------------------
            elif self.state == "RETURN":
                dx = self.start_x - self.x
                dy = self.start_y - self.y
                dist_s = math.sqrt(dx * dx + dy * dy)

                if dist_s <= self.MEASURE_OFFSET + 0.02:
                    self.stop()
                    self.ev3.speaker.beep()
                    self.say("AT START")
                    self.state = "DONE"
                    continue

                bearing = math.atan2(dy, dx)
                err = wrap_pi(bearing - self.theta)  # CCW positive
                # target is CW (err<0) -> steer right (positive turn)
                turn = clamp(-3.0 * math.degrees(err), -60, 60)
                speed = clamp((dist_s - self.MEASURE_OFFSET) * 500, 60, self.RETURN_SPEED)
                self.steer(speed, turn)


def main():
    lab3 = MyController()
    lab3.run()


if __name__ == "__main__":
    main()