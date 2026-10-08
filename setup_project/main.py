#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
# motor1 = Motor(Port.A, positive_direction = Direction.COUNTERCLOCKWISE)
touch_sensor1 = TouchSensor(port = Port.S1)
touch_sensor2 = TouchSensor(port = Port.S4)
# us_sensor = UltrasonicSensor(port = Port.S3)
# gyro_sensor = GyroSensor(port = Port.S2)

# Write your program here.
ev3.speaker.beep()
while (True):
    ev3.screen.clear()
    # ev3.screen.print("Distance:", us_sensor.distance(), "mm")
    # ev3.screen.print("Gyro:", gyro_sensor.angle(), "deg")
    # ev3.screen.print("Gyro Speed:", gyro_sensor.speed(), "deg/s")
    if touch_sensor1.pressed() and touch_sensor2.pressed():
        ev3.speaker.say("Both sensors are pressed")
        print("Both sensors are pressed")
    elif touch_sensor1.pressed():
        ev3.speaker.say("Sensor 1 is pressed")
        print("Sensor 1 is pressed")
    elif touch_sensor2.pressed():
        ev3.speaker.say("Sensor 2 is pressed")
        print("Sensor 2 is pressed")
    else:
        ev3.speaker.say("No sensors are pressed")
        print("No sensors are pressed")
    wait(100)