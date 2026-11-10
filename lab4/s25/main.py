#!/usr/bin/env pybricks-micropython
# Authors: Kevin Parris (730605045), Jenny Nguyen (730605045)
#
# Lab 4: Bug Race
# Algorithm: Bug2 — approach goal on M-line, wall-follow on obstacle contact,
# leave wall when M-line is crossed closer to goal than the hit point.

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import SoundFile, ImageFile
import math


class MyController:
    def __init__(self):
        self.ev3         = EV3Brick()
        self.left_motor  = Motor(Port.D)
        self.right_motor = Motor(Port.C)
        self.ultrasonic  = UltrasonicSensor(Port.S1)
        self.touch1      = TouchSensor(Port.S3)
        self.touch2      = TouchSensor(Port.S2)
        self.gyro        = GyroSensor(Port.S4)
        wait(1000)

        self.song = [
        "E4", "G4", "A4", "A4",
        "G4", "E4", "D4",
        "E4", "G4", "A4", "A4",
        "G4", "E4", "D4",
        "E4", "G4", "A4", "C5",
        "A4", "G4", "E4"
        ]
        self.durations = [
            200, 200, 300, 300,
            200, 200, 400,
            200, 200, 300, 300,
            200, 200, 400,
            200, 200, 300, 400,
            200, 200, 500
        ]
        self.note_index = 0
        self.note_timer = StopWatch()
        self.NOTE_DURATION = 300  

        print("RAW SONIC:", self.ultrasonic.distance())

        # self.WHEEL_RADIUS = 0.03175
        self.WHEEL_RADIUS = 0.0275
        self.WHEEL_BASE   = 0.119
        # self.WHEEL_BASE   = 0.127

        self.START_X = 0.5
        self.START_Y = 0.0
        self.GOAL_X  = 3.0
        self.GOAL_Y  = 3.0

        self.GOAL_RADIUS     = 0.03
        self.WORKSPACE_MIN_X = 0.0
        self.WORKSPACE_MAX_X = 4.0
        self.WORKSPACE_MIN_Y = 0.0
        self.WORKSPACE_MAX_Y = 4.0
        self.BOUNDARY_MARGIN = 0.15
       
        self.APPROACH_SPEED       = 250
        self.APPROACH_MED_SPEED   = 230
        self.APPROACH_SLOW_SPEED  = 200
        self.APPROACH_CREEP_SPEED = 180
        self.TURN_SPEED           = 180
        self.FORWARD_SPEED        = 300
        self.REVERSE_SPEED        = 120
        
        self.DRIVE_BIAS           = 1.5
        
        turn_arc          = (math.pi / 2.0) * (self.WHEEL_BASE / 2.0)
        self.TURN_DEG_90  = 1.05 * math.degrees(turn_arc / self.WHEEL_RADIUS)
        self.TURN_DEG_MIN = self.TURN_DEG_90 / 3.0
        
        self.OBSTACLE_DETECT_DIST = 0.28
        self.sonic_armed          = False
       
        # self.TOO_CLOSE_THRESHOLD = 0.1
        self.TOO_CLOSE_THRESHOLD = 0.20
        # self.TOO_FAR_THRESHOLD   = 0.45
        self.TOO_FAR_THRESHOLD = 0.45
        self.STRAIGHT_LOW        = 0.21
        self.STRAIGHT_HIGH       = 0.40
        # self.STRAIGHT_HIGH       = 0.40
        # self.reference_dist      = 0.25
        self.reference_dist = 0.18
        self.wall_mode           = "STRAIGHT"
        self.too_far_counter     = 0
        self.too_far_steps       = 4
    
        self.hit_x            = 0.0
        self.hit_y            = 0.0
        self.hit_dist_to_goal = 0.0
        self.hit_proj         = 0.0
        self.follow_dist      = 0.0
        self.obstacle_count   = 0

        self.MIN_FOLLOW_BEFORE_MLINE = 0.45
        self.MLINE_BAND              = 0.12

        
        self.perp_sign     = 0
        self.mline_crossed = False
        
        dx_ml  = self.GOAL_X - self.START_X
        dy_ml  = self.GOAL_Y - self.START_Y
        len_ml = math.sqrt(dx_ml * dx_ml + dy_ml * dy_ml)

        self.mline_nx = dy_ml / len_ml
        self.mline_ny =  -dx_ml / len_ml
        
        self.just_left_wall   = False
        self.leave_wall_x     = 0.0
        self.leave_wall_y     = 0.0
        self.LEAVE_DIST_GUARD = 0.22
        # FIX: Step timeout so sonic_armed is re-enabled even if the robot never
        # physically travels LEAVE_DIST_GUARD after leaving the wall.
        self.leave_wall_steps     = 0
        self.LEAVE_WALL_MAX_STEPS = 200
        
        self.x     = self.START_X
        self.y     = self.START_Y
        self.theta = math.pi / 2.0
        self.prev_left  = 0
        self.prev_right = 0
       
        self.turn_start_left  = 0
        self.turn_start_right = 0
        self.reverse_counter  = 0
        self.REVERSE_STEPS    = 100
        self.settle_counter   = 0
        
        self.last_progress_x   = self.START_X
        self.last_progress_y   = self.START_Y
        self.progress_timer    = 0
        self.PROGRESS_INTERVAL = 150
        self.MIN_PROGRESS_DIST = 0.06
        self.escape_counter    = 0
        
        self.stuck_watch    = StopWatch()
        self.last_stuck_x   = self.START_X
        self.last_stuck_y   = self.START_Y
        self.STUCK_TIME_MS  = 10000
        self.STUCK_MIN_MOVE = 0.02
        
        self.best_dist_to_goal       = 999.0
        self.has_been_near_goal      = False
        self.NEAR_GOAL_MEMORY_RADIUS = 0.35
        self.PASSED_GOAL_STOP_RADIUS = 0.35
        
        self.state            = "WAITING"
        self.last_screen_mode = None
        self.loop_print_ctr   = 0
        
        self.debug_watch = StopWatch()

        self.last_pos_debug_ms      = 0
        self.last_mline_debug_ms    = 0
        self.last_follow_debug_ms   = 0
        self.last_approach_debug_ms = 0

        self.POS_DEBUG_INTERVAL_MS      = 3000
        self.MLINE_DEBUG_INTERVAL_MS    = 1000
        self.FOLLOW_DEBUG_INTERVAL_MS   = 6000
        self.APPROACH_DEBUG_INTERVAL_MS = 3000
        
    def play_music_step(self):
        if self.note_timer.time() >= self.durations[self.note_index]:
            note_map = {
                "C4": 262,
                "D4": 294,
                "E4": 330,
                "F4": 349,
                "G4": 392,
                "A4": 440,
                "B4": 494,
                "C5": 523
}
    

            note = self.song[self.note_index]
            freq = note_map.get(note, 440)

            self.ev3.speaker.beep(freq, 50)

            self.note_index = (self.note_index + 1) % len(self.song)
            self.note_timer.reset()

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def touch_pressed(self):
        return self.touch1.pressed() or self.touch2.pressed()

    def show_once(self, label, val=0.0):
        if self.last_screen_mode != label:
            self.ev3.screen.clear()
            self.ev3.screen.print(label)
            self.ev3.screen.print("{:.3f}".format(val))
            self.last_screen_mode = label

    def compute_perp(self):
        dx = self.x - self.START_X
        dy = self.y - self.START_Y
        return self.mline_nx * dx + self.mline_ny * dy

    def update_odometry(self):
        cur_left  = self.left_motor.angle()
        cur_right = self.right_motor.angle()

        d_left  = math.radians(cur_left  - self.prev_left)  * self.WHEEL_RADIUS
        d_right = math.radians(cur_right - self.prev_right) * self.WHEEL_RADIUS

        self.prev_left  = cur_left
        self.prev_right = cur_right

        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / self.WHEEL_BASE

        self.x     += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y     += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Compute which side of the M-line we are on this tick.
        perp     = self.compute_perp()
        if not hasattr(self, 'prev_perp'):
            self.prev_perp = perp
            self.mline_crossed = False
            return abs(d_center)

        self.mline_crossed = (self.prev_perp * perp < 0)
        self.prev_perp = perp
        return abs(d_center)

    def dist_to_goal(self):
        dx = self.GOAL_X - self.x
        dy = self.GOAL_Y - self.y
        return math.sqrt(dx * dx + dy * dy)

    def goal_projection(self):
        dx    = self.x - self.START_X
        dy    = self.y - self.START_Y
        dx_ml = self.GOAL_X - self.START_X
        dy_ml = self.GOAL_Y - self.START_Y
        return (dx * dx_ml + dy * dy_ml) / (dx_ml * dx_ml + dy_ml * dy_ml)

    def get_mline_debug_values(self):
        perp = self.compute_perp()
        proj = self.goal_projection()

        # Primary: sign-flip detects crossing even when drift keeps perp from reading zero.
        # Fallback: proximity band catches the case where accumulated odometry error is
        # large enough that the computed perp never changes sign despite physical crossing.
        on_ml         = (self.mline_crossed or abs(perp) < self.MLINE_BAND) and proj > 0.05
        min_follow_ok = self.follow_dist >= self.MIN_FOLLOW_BEFORE_MLINE
        closer_ok     = self.dist_to_goal() < self.hit_dist_to_goal - 0.05
        proj_past_hit = proj > self.hit_proj + 0.08

        return perp, proj, on_ml, min_follow_ok, closer_ok, proj_past_hit

    def heading_error_to_goal(self):
        target = math.atan2(self.GOAL_Y - self.y, self.GOAL_X - self.x)
        err    = target - self.theta
        while err >  math.pi: err -= 2.0 * math.pi
        while err < -math.pi: err += 2.0 * math.pi
        return err

    def heading_correction(self):
        err    = self.heading_error_to_goal()

        corr = 140 * math.sin(err)

        return max(min(corr, 120), -120)
        # d_goal = self.dist_to_goal()

        # if d_goal < 0.50:
        #     gain, max_corr = 70.0, 55.0
        # elif d_goal < 1.00:
        #     gain, max_corr = 85.0, 65.0
        # else:
        #     gain, max_corr = 100.0, 80.0

        # corr = gain * (err / (math.pi / 2.0))
        # return max(min(corr, max_corr), -max_corr)
    

    def approach_speed_for_goal_distance(self):
        d_goal = self.dist_to_goal()
        if d_goal < 0.27:
            return self.APPROACH_CREEP_SPEED
        elif d_goal < 0.35:
            return self.APPROACH_SLOW_SPEED
        elif d_goal < 0.80:
            return self.APPROACH_MED_SPEED
        else:
            return self.APPROACH_SPEED

    def check_goal_reached_or_passed(self):
        d_goal = self.dist_to_goal()
        proj   = self.goal_projection()

        if d_goal < self.best_dist_to_goal:
            self.best_dist_to_goal = d_goal
        if d_goal <= self.NEAR_GOAL_MEMORY_RADIUS:
            self.has_been_near_goal = True

        if d_goal <= self.GOAL_RADIUS:
            
            print("x={:.3f} y={:.3f} d2g={:.3f} proj={:.2f}".format(
                self.x, self.y, d_goal, proj))
            
            return True

        if self.has_been_near_goal and proj >= 1.00 and d_goal <= self.PASSED_GOAL_STOP_RADIUS:

            print("x={:.3f} y={:.3f} d2g={:.3f} best_d2g={:.3f} proj={:.2f}".format(
                self.x, self.y, d_goal, self.best_dist_to_goal, proj))
    
            return True

        return False

    def check_workspace_guard(self):
        if self.goal_projection() < 0.05:
            return False

        near_left   = self.x < self.WORKSPACE_MIN_X - 0.10
        near_right  = self.x > self.WORKSPACE_MAX_X - self.BOUNDARY_MARGIN
        near_bottom = self.y < self.WORKSPACE_MIN_Y - 0.10
        near_top    = self.y > self.WORKSPACE_MAX_Y - self.BOUNDARY_MARGIN

        if near_left or near_right or near_bottom or near_top:
            print("state={} x={:.3f} y={:.3f} d2g={:.3f}".format(
                self.state, self.x, self.y, self.dist_to_goal()))
            
            return True

        return False

    def reset_stuck_timer(self):
        self.stuck_watch.reset()
        self.last_stuck_x = self.x
        self.last_stuck_y = self.y

    def check_time_stuck(self):
        dx = self.x - self.last_stuck_x
        dy = self.y - self.last_stuck_y
        if math.sqrt(dx * dx + dy * dy) >= self.STUCK_MIN_MOVE:
            self.last_stuck_x = self.x
            self.last_stuck_y = self.y
            self.stuck_watch.reset()
            return False
        return self.stuck_watch.time() >= self.STUCK_TIME_MS

    def start_wall_follow(self, dist):
        self.reference_dist   = dist if 0.15 < dist < 0.45 else 0.25
        self.wall_mode        = "STRAIGHT"
        self.last_screen_mode = None
        self.follow_dist      = 0.0
        self.too_far_counter  = 0
        self.last_progress_x  = self.x
        self.last_progress_y  = self.y
        self.progress_timer   = 0
        self.prev_left        = self.left_motor.angle()
        self.prev_right       = self.right_motor.angle()
        # FIX: Wrapped in try/except -- Webots mock may not implement reset_angle.
        # Print confirms the reset is actually working in sim.
        try:
            self.gyro.reset_angle(0)
            print("gyro reset OK in start_wall_follow: angle={}".format(self.gyro.angle()))
        except Exception as e:
            print("gyro reset_angle not supported in start_wall_follow: {}".format(e))
        self.reset_stuck_timer()

        # Reseed the crossing detector to the current side of the M-line.
        # This prevents the detector from immediately firing on entry, and
        # ensures it only fires when the robot actually crosses to the other side.
        perp           = self.compute_perp()
        self.perp_sign = 1 if perp >= 0 else -1
        self.mline_crossed = False

        print("obs={} ref={:.3f}m hit=({:.3f},{:.3f}) hit_d2g={:.3f} hit_proj={:.3f}".format(
            self.obstacle_count, self.reference_dist,
            self.hit_x, self.hit_y,
            self.hit_dist_to_goal, self.hit_proj))
        print("starting_perp={:.3f} perp_sign={}".format(perp, self.perp_sign))
        
    def finish_run(self, reason):
        self.stop()
        
        print(reason)
        print("x={:.3f} y={:.3f} theta={:.2f} d2g={:.3f} proj={:.2f}".format(
            self.x, self.y, self.theta,
            self.dist_to_goal(), self.goal_projection()))
        
        self.ev3.speaker.beep(frequency=1200, duration=350)
        wait(150)
        self.ev3.speaker.beep(frequency=1500, duration=350)
        self.state = "DONE"

    # ====================================================================================
    # Debug printing
    # ====================================================================================

    def print_position_debug(self, dist):
        now_ms = self.debug_watch.time()
        if now_ms - self.last_pos_debug_ms >= self.POS_DEBUG_INTERVAL_MS:
            self.last_pos_debug_ms = now_ms
            self.ev3.screen.print("state={}".format(self.state))
            self.ev3.screen.print("x={:.3f} y={:.3f}".format(self.x, self.y))
            self.ev3.screen.print("theta={:.2f} d2g={:.3f} proj={:.2f} sonic={:.3f}".format(self.theta,
                self.dist_to_goal(), self.goal_projection(), dist))

    def print_approach_debug(self, dist, corr, speed):
        now_ms = self.debug_watch.time()
        if now_ms - self.last_approach_debug_ms >= self.APPROACH_DEBUG_INTERVAL_MS:
            self.last_approach_debug_ms = now_ms
            err = self.heading_error_to_goal()
            
            print("x={:.3f} y={:.3f} theta={:.2f} d2g={:.3f} proj={:.2f}".format(
                self.x, self.y, self.theta,
                self.dist_to_goal(), self.goal_projection()))
            print("sonic={:.3f} armed={} just_left={}".format(
                dist, self.sonic_armed, self.just_left_wall))
            print("heading_err={:.2f} corr={:.1f} speed={}".format(err, corr, speed))
            

    def print_follow_debug(self, dist):
        now_ms = self.debug_watch.time()
        if now_ms - self.last_follow_debug_ms >= self.FOLLOW_DEBUG_INTERVAL_MS:
            self.last_follow_debug_ms = now_ms
            perp, proj, on_ml, min_follow_ok, closer_ok, proj_past_hit = self.get_mline_debug_values()
            
            print("obs={} mode={} x={:.3f} y={:.3f} theta={:.2f}".format(
                self.obstacle_count, self.wall_mode,
                self.x, self.y, self.theta))
            print("sonic={:.3f} follow_dist={:.2f} d2g={:.3f} hit_d2g={:.3f}".format(
                dist, self.follow_dist,
                self.dist_to_goal(), self.hit_dist_to_goal))
            print("perp={:.3f} perp_sign={} crossed={} proj={:.2f} hit_proj={:.2f}".format(
                perp, self.perp_sign, self.mline_crossed, proj, self.hit_proj))
            print("on_ml={} min_follow_ok={} closer_ok={} proj_past_hit={}".format(
                on_ml, min_follow_ok, closer_ok, proj_past_hit))
           

    def print_mline_perp(self):
        now_ms = self.debug_watch.time()
        if now_ms - self.last_mline_debug_ms >= self.MLINE_DEBUG_INTERVAL_MS:
            self.last_mline_debug_ms = now_ms
            perp, proj, on_ml, min_follow_ok, closer_ok, proj_past_hit = self.get_mline_debug_values()
            print("perp={:.3f} sign={} crossed={} proj={:.2f} on_ml={} min_ok={} closer={} past_hit={}".format(
                perp, self.perp_sign, self.mline_crossed, proj,
                on_ml, min_follow_ok, closer_ok, proj_past_hit))


    def run(self):
        
        while True:
      
            wait(20)
            dist = self.ultrasonic.distance() / 1000.0
            self.play_music_step()

            if self.state == "WAITING":
                self.stop()
                self.ev3.screen.clear()
                self.ev3.screen.print("Press CENTER")

                if Button.CENTER in self.ev3.buttons.pressed():
                    self.x     = self.START_X
                    self.y     = self.START_Y
                    self.theta = math.pi / 2.0

                    self.prev_left  = self.left_motor.angle()
                    self.prev_right = self.right_motor.angle()

                    # FIX: Wrapped in try/except -- Webots mock may not implement reset_angle.
                    try:
                        self.gyro.reset_angle(0)
                        print("gyro reset OK in WAITING: angle={}".format(self.gyro.angle()))
                    except Exception as e:
                        print("gyro reset_angle not supported in WAITING: {}".format(e))

                    self.wall_mode        = "STRAIGHT"
                    self.last_screen_mode = None
                    self.follow_dist      = 0.0
                    self.obstacle_count   = 0
                    self.just_left_wall   = False
                    self.leave_wall_steps = 0
                    self.sonic_armed      = False

                    self.best_dist_to_goal  = 999.0
                    self.has_been_near_goal = False

                    self.perp_sign     = 0
                    self.mline_crossed = False

                    self.debug_watch.reset()
                    self.last_pos_debug_ms      = 0
                    self.last_mline_debug_ms    = 0
                    self.last_follow_debug_ms   = 0
                    self.last_approach_debug_ms = 0

                    
                    print("start ({:.2f},{:.2f}) -> goal ({:.2f},{:.2f})".format(
                        self.x, self.y, self.GOAL_X, self.GOAL_Y))
                

                    self.reset_stuck_timer()
                    self.state = "APPROACH_GOAL"

           
            elif self.state == "APPROACH_GOAL":
                self.update_odometry()
                self.print_position_debug(dist)

                if self.check_goal_reached_or_passed():
                    self.finish_run("Goal reached or passed safely.")
                    continue

                if self.check_workspace_guard():
                    self.finish_run("Workspace boundary guard triggered.")
                    continue

                if self.check_time_stuck():
                    self.stop()
                    
                    print("no movement for 10s at ({:.3f},{:.3f})".format(self.x, self.y))
                    
                    self.obstacle_count  += 1
                    self.hit_x            = self.x
                    self.hit_y            = self.y
                    self.hit_dist_to_goal = self.dist_to_goal()
                    self.hit_proj         = self.goal_projection()
                    self.reverse_counter  = 0
                    self.reset_stuck_timer()
                    self.state = "REVERSE"
                    continue

                # FIX: Added step-counter timeout alongside the distance check so
                # sonic_armed is always re-enabled even if the robot never travels
                # LEAVE_DIST_GUARD (e.g. stalls or turns in place after leaving the wall).
                if self.just_left_wall:
                    self.leave_wall_steps += 1
                    ldx        = self.x - self.leave_wall_x
                    ldy        = self.y - self.leave_wall_y
                    leave_dist = math.sqrt(ldx * ldx + ldy * ldy)
                    if leave_dist >= self.LEAVE_DIST_GUARD or \
                       self.leave_wall_steps >= self.LEAVE_WALL_MAX_STEPS:
                        self.just_left_wall   = False
                        self.leave_wall_steps = 0
                        self.sonic_armed      = True
                        print("sonic re-armed after leaving wall (dist={:.3f} steps={})".format(
                            leave_dist, self.leave_wall_steps))

                if self.touch_pressed():
                    self.stop()
                    self.obstacle_count  += 1
                    self.hit_x            = self.x
                    self.hit_y            = self.y
                    self.hit_dist_to_goal = self.dist_to_goal()
                    self.hit_proj         = self.goal_projection()
                    self.sonic_armed      = True
                    
                    print("obstacle {} touch at ({:.3f},{:.3f}) d2g={:.3f} proj={:.3f}".format(
                        self.obstacle_count,
                        self.hit_x, self.hit_y,
                        self.hit_dist_to_goal, self.hit_proj))
                    
                    self.reverse_counter = 0
                    self.state = "REVERSE"
                    continue

                if self.sonic_armed and dist < self.OBSTACLE_DETECT_DIST:
                    self.stop()
                    self.obstacle_count  += 1
                    self.hit_x            = self.x
                    self.hit_y            = self.y
                    self.hit_dist_to_goal = self.dist_to_goal()
                    self.hit_proj         = self.goal_projection()
                    
                    print("obstacle {} sonic at ({:.3f},{:.3f}) d2g={:.3f} proj={:.3f} sonic={:.3f}".format(
                        self.obstacle_count,
                        self.hit_x, self.hit_y,
                        self.hit_dist_to_goal, self.hit_proj, dist))
                    
                    self.turn_start_left  = self.left_motor.angle()
                    self.turn_start_right = self.right_motor.angle()
                    
                    try:
                        self.gyro.reset_angle(0)
                        print("gyro reset OK pre-turn: angle={}".format(self.gyro.angle()))
                    except Exception as e:
                        print("gyro reset_angle not supported pre-turn: {}".format(e))
                    self.state = "TURN_RIGHT"
                    continue

                if not self.sonic_armed and not self.just_left_wall:
                    self.sonic_armed = True

                corr           = self.heading_correction()
                approach_speed = self.approach_speed_for_goal_distance()

                self.print_approach_debug(dist, corr, approach_speed)

                self.left_motor.run(approach_speed - corr)
                self.right_motor.run(approach_speed + corr)

                self.ev3.screen.clear()
                self.ev3.screen.print("TO GOAL")
                self.ev3.screen.print("{:.2f}m".format(self.dist_to_goal()))

           
            elif self.state == "REVERSE":
                self.update_odometry()
                self.left_motor.run(-self.REVERSE_SPEED)
                self.right_motor.run(-self.REVERSE_SPEED)
                self.reverse_counter += 1
                if self.reverse_counter >= self.REVERSE_STEPS:
                    self.stop()
                    self.turn_start_left  = self.left_motor.angle()
                    self.turn_start_right = self.right_motor.angle()
                    # FIX: Wrapped in try/except -- Webots mock may not implement reset_angle.
                    try:
                        self.gyro.reset_angle(0)
                        print("gyro reset OK post-reverse: angle={}".format(self.gyro.angle()))
                    except Exception as e:
                        print("gyro reset_angle not supported post-reverse: {}".format(e))
                    self.state = "TURN_RIGHT"

            
            elif self.state == "TURN_RIGHT":
                self.update_odometry()
                left_change  = abs(self.left_motor.angle()  - self.turn_start_left)
                right_change = abs(self.right_motor.angle() - self.turn_start_right)
                avg_turn     = (left_change + right_change) / 2.0

                turned_min    = avg_turn >= self.TURN_DEG_MIN
                wall_in_range = 0.15 < dist < 0.40
                turned_max    = avg_turn >= self.TURN_DEG_90

                if not turned_min or (not wall_in_range and not turned_max):
                    self.left_motor.run(self.TURN_SPEED)
                    self.right_motor.run(-self.TURN_SPEED)
                else:
                    self.stop()
                    wait(400)
                    self.start_wall_follow(dist)
                    wait(200)
                    self.settle_counter = 0
                    self.state = "SETTLE"


            elif self.state == "SETTLE":
                self.left_motor.run(self.FORWARD_SPEED)
                self.right_motor.run(self.FORWARD_SPEED)
                self.settle_counter += 1
                if self.settle_counter > 20:
                    print("")
                    print("========== ENTERING FOLLOW ==========")
                    print("x={:.3f} y={:.3f} d2g={:.3f}".format(
                        self.x, self.y, self.dist_to_goal()))
                    print("=====================================")
                    print("")
                    self.state = "FOLLOW"

            
            elif self.state == "FOLLOW":
                step = self.update_odometry()

                self.follow_dist    += step
                self.progress_timer += 1

                self.print_position_debug(dist)
                self.print_follow_debug(dist)
                self.print_mline_perp()

                if self.check_goal_reached_or_passed():
                    self.finish_run("Goal reached while following wall.")
                    continue

                if self.check_workspace_guard():
                    self.finish_run("Workspace boundary guard triggered while following.")
                    continue

                if self.touch_pressed():
                    self.stop()
                    self.obstacle_count  += 1
                    self.hit_x            = self.x
                    self.hit_y            = self.y
                    self.hit_dist_to_goal = self.dist_to_goal()
                    self.hit_proj         = self.goal_projection()
                    
                    print("obs={} touch at ({:.3f},{:.3f}) d2g={:.3f}".format(
                        self.obstacle_count, self.hit_x, self.hit_y, self.hit_dist_to_goal))
                    
                    self.reverse_counter = 0
                    self.reset_stuck_timer()
                    self.state = "REVERSE"
                    continue

                if self.check_time_stuck():
                    self.stop()
                    
                    print("no movement for 10s at ({:.3f},{:.3f})".format(self.x, self.y))
                    
                    self.escape_counter = 0
                    self.reset_stuck_timer()
                    self.state = "ESCAPE"
                    continue

                if self.progress_timer >= self.PROGRESS_INTERVAL:
                    dx = self.x - self.last_progress_x
                    dy = self.y - self.last_progress_y
                    if math.sqrt(dx * dx + dy * dy) < self.MIN_PROGRESS_DIST:
                        
                        print("stuck at ({:.3f},{:.3f}), escaping".format(self.x, self.y))
                        
                        self.escape_counter = 0
                        self.state = "ESCAPE"
                        continue
                    self.last_progress_x = self.x
                    self.last_progress_y = self.y
                    self.progress_timer  = 0

                perp, proj, on_ml, min_follow_ok, closer_ok, proj_past_hit = self.get_mline_debug_values()

                # Leave the wall when all four conditions hold:
                #   1. Traveled enough distance along the wall
                #   2. Sign-flip crossing detected this tick (robust to odometry drift)
                #   3. Closer to goal than the hit point
                #   4. Projection past the hit point — on the far side of the obstacle
                if min_follow_ok and on_ml and closer_ok and proj_past_hit:
                    self.stop()
                    self.ev3.speaker.beep(frequency=1000, duration=300)
                    wait(500)
                    self.ev3.screen.clear()

                    # print(">>> LEAVING WALL ON M-LINE <<<")
                    # print("obs={} x={:.3f} y={:.3f} d2g={:.3f} hit_d2g={:.3f}".format(
                    #     self.obstacle_count,
                    #     self.x, self.y,
                    #     self.dist_to_goal(), self.hit_dist_to_goal))
                    # print("perp={:.3f} proj={:.2f} hit_proj={:.2f} follow_dist={:.2f}".format(
                    #     perp, proj, self.hit_proj, self.follow_dist))
            
                    wait(200)
                    self.just_left_wall   = True
                    self.leave_wall_steps = 0
                    self.sonic_armed      = False
                    self.wall_mode        = "STRAIGHT"
                    self.last_screen_mode = None
                    self.state = "REORIENT"
                    continue

                # ------------------------- Wall mode transitions -------------------------

                if self.wall_mode == "STRAIGHT":
                    if dist < self.TOO_CLOSE_THRESHOLD:
                        self.wall_mode = "TOO_CLOSE"
                    elif dist > self.TOO_FAR_THRESHOLD:
                        self.wall_mode       = "TOO_FAR"
                        self.too_far_counter = 0

                elif self.wall_mode == "TOO_CLOSE":
                    if dist >= self.STRAIGHT_LOW:
                        self.wall_mode = "STRAIGHT"

                elif self.wall_mode == "TOO_FAR":
                    if self.too_far_counter >= self.too_far_steps or dist <= self.STRAIGHT_HIGH:
                        self.wall_mode       = "STRAIGHT"
                        self.too_far_counter = 0

                # -------------------------- Wall motor commands -------------------------

                if self.wall_mode == "STRAIGHT":
                    self.show_once("STRAIGHT", dist)
                    self.left_motor.run(self.FORWARD_SPEED)
                    # FIX: Use named DRIVE_BIAS constant instead of magic 1.5 literal.
                    self.right_motor.run(self.FORWARD_SPEED + self.DRIVE_BIAS)

                elif self.wall_mode == "TOO_CLOSE":
                    self.show_once("TOO_CLOSE", dist)
                    g    = self.gyro.angle()
                    err  = self.reference_dist - dist
                    turn = 1500 * err * max(0.2, 1 - abs(g) / 45.0)
                    turn = max(min(turn, 60), 10)
                    if abs(g) > 30:
                        turn *= 0.9
                    self.left_motor.run(80 + turn)
                    self.right_motor.run(80 - turn)

                elif self.wall_mode == "TOO_FAR":
                    self.too_far_counter += 1
                    self.show_once("TOO_FAR", dist)
                    g = self.gyro.angle()
                    if dist > 2.0:
                        turn = 8
                    elif dist > 1.0:
                        turn = 60
                    else:
                        err  = dist - self.reference_dist
                        # turn = 600 * err if err < 0.05 else 1000 * err
                        turn = 1400 * err
                        if g > 18:   turn *= 0.7
                        elif g > 10: turn *= 0.8
                        turn = max(min(turn, 30), 0)

                    self.left_motor.run(80 - turn)
                    self.right_motor.run(80 + turn)
                    if self.too_far_counter >= 60 or dist <= self.STRAIGHT_HIGH:
                        self.wall_mode       = "STRAIGHT"
                        self.too_far_counter = 0


            elif self.state == "REORIENT":
                self.update_odometry()
                err = self.heading_error_to_goal()

                if abs(err) > 0.10:
                    if err > 0:
                        self.left_motor.run(-self.TURN_SPEED)
                        self.right_motor.run(self.TURN_SPEED)
                    else:
                        self.left_motor.run(self.TURN_SPEED)
                        self.right_motor.run(-self.TURN_SPEED)
                else:
                    self.stop()
                    wait(200)
                    self.leave_wall_x = self.x
                    self.leave_wall_y = self.y
                    
                    print("x={:.3f} y={:.3f} theta={:.2f} err={:.3f}".format(
                        self.x, self.y, self.theta, err))
                
                    
                    self.reset_stuck_timer()
                    self.perp_sign = 0
                    self.mline_crossed = False
                    self.state = "APPROACH_GOAL"

          
            elif self.state == "ESCAPE":
                self.escape_counter += 1
                if self.escape_counter <= 50:
                    self.left_motor.run(-self.REVERSE_SPEED)
                    self.right_motor.run(-self.REVERSE_SPEED)
                elif self.escape_counter <= 90:
                    self.left_motor.run(-self.TURN_SPEED)
                    self.right_motor.run(self.TURN_SPEED)
                else:
                    self.stop()
                    self.wall_mode        = "STRAIGHT"
                    self.last_screen_mode = None
                    self.too_far_counter  = 0
                    self.last_progress_x  = self.x
                    self.last_progress_y  = self.y
                    self.progress_timer   = 0
                    self.reset_stuck_timer()
                    self.state = "FOLLOW"

            elif self.state == "DONE":
                self.stop()
                self.ev3.screen.clear()
                self.ev3.screen.print("DONE!")
                self.ev3.screen.print("({:.2f},{:.2f})".format(self.x, self.y))


def main():
    lab4 = MyController()
    lab4.run()


if __name__ == "__main__":
    main()