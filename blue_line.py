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

#define your variables
ev3 = EV3Brick()
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
medium_motor = Motor(Port.A)
large_motor = Motor(Port.D)
wheel_diameter = 56
axle_track = 115  
line_sensor = ColorSensor(Port.S2)
line_sensor1 = ColorSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

#moves robot forward 700 millimeters towards the mission
robot.straight(700)
ev3.speaker.beep()
#pushes the step counter slowly
robot.straight(100)
Stop.BRAKE
ev3.speaker.beep()

# goes back for targetting the blue part of the step counter
robot.straight(-50)
robot.straight(10)
Stop.BRAKE
ev3.speaker.beep()

#does its final push
robot.straight(150)
Stop.BRAKE
ev3.speaker.beep()

#goes back so it can be easy to detect the first black line
robot.straight(-200)

#uses the color sensor (2) to allow it to detect the line
while True:
  
    robot.drive(-30,0)
    print(line_sensor.reflection())
    if line_sensor.reflection() <= 10:
        robot.stop(Stop.BRAKE)
        break

#turns towards the pull up bar
left_motor.run_angle(450,590)

while True:
  
    robot.drive(-30,0)
    print(line_sensor1.reflection())
    if line_sensor1.reflection() <= 10:
        robot.stop(Stop.BRAKE)
        break
ev3.speaker.beep()

right_motor.run_angle(200,1000)

while True:
    right_motor.run_angle(200,10)
    if line_sensor.reflection() <= 12:
        robot.stop(Stop.BRAKE)
        break
ev3.speaker.beep()
right_motor.run_angle(200,100)
left_motor.run_angle(200,200)

"""
left_motor.run_angle(500,250)
"""

BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2

# Start following the line endlessly.
while True:
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # You can wait for a short time or do other things in this loop.
    wait(10)
 
if robot.distance() == 531: 
    runWhile = False