#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import SoundFile, ImageFile
import math

class MyController:
    def __init__(self):
        # Initialize robot, sensors, and motors
        self.ev3 = EV3Brick()
        self.left_motor = Motor(Port.C)
        self.right_motor = Motor(Port.D)
        self.touch_sensor1 = TouchSensor(Port.S3)
        self.touch_sensor2 = TouchSensor(Port.S2)
        self.ultrasonic_sensor = UltrasonicSensor(Port.S1)
        self.gyro_sensor = GyroSensor(Port.S4)
        wait(1000)

        self.state = "WAITING"

        self.WHEEL_RADIUS = 0.03175
        self.WHEEL_BASE = 0.127

        self.APPROACH_SPEED = 200
        self.REVERSE_SPEED = 100
        self.TURN_SPEED = 200
        self.FORWARD_SPEED = 80
        

        self.REVERSE_STEPS = 90  

        turn_arc = (math.pi / 2.0) * (self.WHEEL_BASE / 2.0)
        self.TURN_DEG_90 = 1 * math.degrees(turn_arc / self.WHEEL_RADIUS)

        self.turn_start_left = 0
        self.turn_start_right = 0
        self.reverse_counter = 0
        self.follow_dist = 0.0
        self.prev_left = self.left_motor.angle()
        self.prev_right = self.right_motor.angle()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.start_x = 0.0
        self.start_y = 0.0

        self.hit_x = 0.0
        self.hit_y = 0.0

        """self.LOOP_ANGLE_MIN = -25
        self.LOOP_ANGLE_MAX = 20
        self.HIT_RETURN_RADIUS = 0.20
        self.OBSTACLE_DETECT_DIST = 0.30
"""

        self.start_saved = False 
        self.hit_saved = False

        self.hit_reached_again = False
        self.start_reached_again = False

        self.clearance_start_left = 0
        self.clearance_start_right = 0
        self.CLEARANCE_DEG = 120   # tune this

        self.reference_dist = 0.0
        self.last_screen_mode = None
        self.wall_mode = "STRAIGHT"
       
    def actuate_motors(self, speed):
        # Helper function for running motors at a given speed
        self.left_motor.run(speed)
        self.right_motor.run(speed+ 1.5)

    def stop(self):
        # Helper function for stopping motors
        self.left_motor.stop()
        self.right_motor.stop()

    def show_once(self, label, dist):
      if self.last_screen_mode != label:
          self.ev3.screen.clear()
          self.ev3.screen.print(label)
          self.ev3.screen.print("{:.3f} m".format(dist))
          print(label, "{:.3f} m".format(dist))
          self.last_screen_mode = label

    def compute_position(self, left_angle, right_angle):
        # Implement diffential drive kinematics to update the robot's position and orientation
        # Hint: Use the formula from the slides
        """cur_left = self.left_motor.angle()
        cur_right = self.right_motor.angle()"""

        delta_left = left_angle - self.prev_left
        delta_right = right_angle - self.prev_right

        self.prev_left = left_angle
        self.prev_right = right_angle

        d_left = math.radians(delta_left) * self.WHEEL_RADIUS
        d_right = math.radians(delta_right) * self.WHEEL_RADIUS

        d_center = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / self.WHEEL_BASE

        self.theta += delta_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        self.follow_dist += abs(d_center)
    
    """def print_current_location(self, label):
        dx = self.x - self.loop_origin_x
        dy = self.y - self.loop_origin_y
        dist_to_loop_origin = math.sqrt(dx * dx + dy * dy)

        self.debug_blue(
            "{} | x={:.3f}, y={:.3f}, theta_deg={:.1f}, gyro_deg={:.1f}, dist_to_loop_origin={:.3f}".format(
                label,
                self.x,
                self.y,
                math.degrees(self.theta),
                self.gyro.angle(),
                dist_to_loop_origin
            )
        )"""

    def distance_to_point(self, px, py):
        return math.sqrt((self.x - px)**2 + (self.y - py)**2)

    def is_near_point(self, px, py, tolerance=0.05):
        return self.distance_to_point(px, py) <= tolerance

    def run(self):
    
         while True:
    
           wait(20)
           
           dist = self.ultrasonic_sensor.distance() / 1000.0
           if self.state == "WAITING":
              self.stop()
              self.ev3.screen.clear()
              self.ev3.screen.print("Press CENTER")


              if Button.CENTER in self.ev3.buttons.pressed():
                  self.x = 0.0
                  self.y = 0.0
                  self.theta = 0.0
                  self.follow_dist = 0.0

                  self.prev_left = self.left_motor.angle()
                  self.prev_right = self.right_motor.angle()

                  self.start_x = self.x
                  self.start_y = self.y
                  self.start_saved = True
                  self.hit_saved = False
                  self.hit_reached_again = False
                  self.start_reached_again = False

                  self.reverse_counter = 0
                  self.wall_mode = "STRAIGHT"
                  self.last_screen_mode = None
                  self.state = "DRIVE"
                  
           elif self.state == "DRIVE":
              self.compute_position(self.left_motor.angle(), self.right_motor.angle())
              self.actuate_motors(self.APPROACH_SPEED)

              if self.touch_sensor1.pressed() or self.touch_sensor2.pressed():
                  self.stop()
                  self.reverse_counter = 0
                  self.state = "REVERSE"


           elif self.state == "REVERSE":
              self.compute_position(self.left_motor.angle(), self.right_motor.angle())
              self.actuate_motors(-self.REVERSE_SPEED)
              self.reverse_counter += 1

              if self.reverse_counter >= self.REVERSE_STEPS:
                  self.stop()
                  self.ev3.speaker.beep()

                  if not self.hit_saved:
                    self.hit_x = self.x
                    self.hit_y = self.y
                    self.hit_saved = True
                  
                  self.turn_start_left = self.left_motor.angle()
                  self.turn_start_right = self.right_motor.angle()
                  # Reset once right before the turn starts.
                  self.gyro_sensor.reset_angle(0)
                  self.follow_start_angle = 0.0
                  self.state = "TURN"
 
           elif self.state == "TURN":
              left_change = abs(self.left_motor.angle() - self.turn_start_left)
              right_change = abs(self.right_motor.angle() - self.turn_start_right)
              turn_amount = (left_change + right_change) / 2.0


              if turn_amount < self.TURN_DEG_90:
                  self.left_motor.run(-self.TURN_SPEED)
                  self.right_motor.run(self.TURN_SPEED)
              else:
                  self.stop()
                  self.reference_dist = dist
                  self.last_screen_mode = None
                  self.show_once("REFERENCE", dist)
                  print("--- REFERENCE DISTANCE SET: {:.3f} m ---".format(self.reference_dist))
                  wait(1500)


                  self.wall_mode = "STRAIGHT"
                  self.last_screen_mode = None


                  self.prev_left = self.left_motor.angle()
                  self.prev_right = self.right_motor.angle()
                  self.follow_dist = 0.0
                  wait(300)


                  self.gyro_sensor.reset_angle(0)

                  self.state = "SETTLE"
                  self.settle_counter = 0

           elif self.state == "SETTLE":
              self.left_motor.run(self.FORWARD_SPEED)
              self.right_motor.run(self.FORWARD_SPEED)


              self.settle_counter += 1
              if self.settle_counter > 20:
  
                  self.state = "FOLLOW"

           elif self.state == "FOLLOW":
               
               dist = self.ultrasonic_sensor.distance() / 1000.0
               self.compute_position(self.left_motor.angle(), self.right_motor.angle())
               #angle = self.gyro_sensor.angle()
               error = self.reference_dist - dist

               if abs(error) < 0.04:
                    self.show_once("STRAIGHT", dist)
                    self.actuate_motors(self.FORWARD_SPEED)
               else:
                    self.show_once("CORRECT", dist)

                    # too close
                    if error > 0:
                        turn = 1500 * error
                        turn = min(turn, 80)

                    else:
                        far_error = -error
                        if far_error < 0.02:
                            turn = -8
                        elif far_error < 0.05:
                            turn = -28
                        else:
                            turn = -30
        
                    turn = max(min(turn, 80), -80)

                    print("DIST: {:.3f} m, REF: {:.3f} m, m ERROR: {:.3f}, TURN: {:.1f}".format(dist, self.reference_dist, error, turn))

                    self.left_motor.run(self.FORWARD_SPEED - turn)
                    self.right_motor.run(self.FORWARD_SPEED + turn)
                
               if self.touch_sensor1.pressed() or self.touch_sensor2.pressed():
                  self.stop()
                  self.reverse_counter = 0
                  self.state = "REVERSE"

               dist_to_hit = self.distance_to_point(self.hit_x, self.hit_y)
               print("follow_dist: {:.3f}, dist_to_hit: {:.3f}, wall_dist: {:.3f}".format(
                    self.follow_dist, dist_to_hit, dist
                ))


               if self.hit_saved and not self.hit_reached_again and self.follow_dist > 1.3:
                    if self.is_near_point(self.hit_x, self.hit_y, tolerance=0.15):
                        self.stop()
                        self.ev3.speaker.beep()
                        print("BACK AT HIT POINT")
                        self.hit_reached_again = True

                        wait(1000)
                        self.turn_start_left = self.left_motor.angle()
                        self.turn_start_right = self.right_motor.angle()

                        
                        self.state = "MOVE_CLEAR"
                        continue
           elif self.state == "MOVE_CLEAR":
            left_change = abs(self.left_motor.angle() - self.clearance_start_left)
            right_change = abs(self.right_motor.angle() - self.clearance_start_right)
            move_amount = (left_change + right_change) / 2.0

            if move_amount < self.CLEARANCE_DEG:
                # move straight a little to create room for the turn
                self.left_motor.run(self.FORWARD_SPEED)
                self.right_motor.run(self.FORWARD_SPEED)
            else:
                self.stop()
                wait(300)
                self.turn_start_left = self.left_motor.angle()
                self.turn_start_right = self.right_motor.angle()
                self.state = "TURN_TO_START"

           elif self.state == "TURN_TO_START":
               left_change = abs(self.left_motor.angle() - self.turn_start_left)
               right_change = abs(self.right_motor.angle() - self.turn_start_right)
               turn_amount = (left_change + right_change) / 2.0

               if turn_amount < self.TURN_DEG_90:
                    self.left_motor.run(self.TURN_SPEED)
                    self.right_motor.run(0)
               else:
                    self.stop()
                    wait(500)
                    self.state = "RETURN_TO_START"

           elif self.state == "RETURN_TO_START":
            self.compute_position(self.left_motor.angle(), self.right_motor.angle())
            self.actuate_motors(self.FORWARD_SPEED)

            if self.start_saved and self.is_near_point(self.start_x, self.start_y, tolerance=0.06):
                self.stop()
                self.ev3.speaker.beep()
                print("BACK AT START POINT")
                break
            

        # make reverse when hit wall and turn 90 deg, continue to follow wall but still remember distance that we have gon 
        # make the tolerance higher? 

def main():
    lab3 = MyController()
    lab3.run()

if __name__ == "__main__":
    main()