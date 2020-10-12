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
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
medium_motor = Motor(Port.A)
front_largeMotor = Motor(Port.D)
wheel_diameter = 56
axle_track = 115

robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

# Initialize the color sensor.
right_sensor = ColorSensor(Port.S2)
left_sensor = ColorSensor(Port.S3)

robot.straight(110)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 110

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2
runWhile = True
# Start following the line endlessly.
while runWhile:
    # Calculate the deviation from the threshold.
    deviation = right_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # set the distance the robot needs to go
    if robot.distance() == 1380:
        runWhile = False

# robot stops after finishing up line following code

robot.stop(Stop.BRAKE)

# the left color sensor senses the perpendicular black line to increase reliability
while True:
  
    robot.drive(100,0)
    print(left_sensor.reflection())
    if left_sensor.reflection() <= 10:
        robot.stop(Stop.BRAKE)
        break

# the robot beeps to confirm the color sensor code worked
ev3.speaker.beep()

# the robot moves backwards just a bit so it has more space to turn
robot.straight(-25)

# robot turns left, forming a right angle
robot.turn(-105)

# robot goes straight as it heads towards the mission
robot.straight(320)

# robot turns right for 150 degrees, with the wheel, and smaller circle in a straight line
robot.turn(155)

# robot goes straight to get closer to the wheel
robot.straight(115)

# the robot turns a little to the left to move the wheel in front of the smaller target
robot.turn(-75)

# the robot moves backwards to give the attachment some space to trap the wheel
robot.straight(-25)

# large motor attachment goes down to trap the wheel in its clutches
front_largeMotor.run_angle(60, 164)

# robot moves backwards to bring wheel outside of the large circle and into the small circle
robot.straight (-100)

# large motor attachment goes back up
front_largeMotor.run_angle(60, -170)

# the robot moves backwards to not ruin any progress that it might have made with the mission
robot.straight(-80)

# robot turns left so that it can go to accomplish the weight machine
robot.turn(-50)

# robot goes straight for a bit before the color sensing code
robot.straight(80)

# left color sensor senses the black line first
while True:
  
    robot.drive(100,0)
    print(left_sensor.reflection())
    if left_sensor.reflection() <= 10:
        robot.stop(Stop.BRAKE)
        break

# robot beeps to confirm that it sensed the black line
ev3.speaker.beep()

# run the right motor to turn a bit
right_motor.run_angle(5,5)

# right color sensor senses the black line while turning with the right motor
while True:
  
    right_motor.run(70)
    print(right_sensor.reflection())
    if right_sensor.reflection() <= 13:
        robot.stop(Stop.BRAKE)
        break

# robot beeps to confirm that it sensed the black line 
ev3.speaker.beep()

# robot turns a little to the left to align itself to the weight machine
robot.turn(-5)

# robot goes straight so the attachment can lower the weight machine
robot.straight(165)

# large motor attachment goes down to lower the weight machine
front_largeMotor.run_angle(60,160)

# robot goes backwards away from the weight machine after completing the mission
robot.straight(-10)

# robot goes backwards while the left color sensor tries to sense the black line
while True:
  
    robot.drive(-100, 0)
    print(left_sensor.reflection())
    if left_sensor.reflection() <= 13:
        robot.stop(Stop.BRAKE)
        break 

# robot stops for accuracy
robot.stop(Stop.BRAKE) 

# robot beeps to indicate the color sensing code worked
ev3.speaker.beep()

# turn with running the right motor a little
right_motor.run_angle(20,5)

# right color sensor senses the black line to square with the line 
while True:
  
    right_motor.run(70)
    print(right_sensor.reflection())
    if right_sensor.reflection() <= 13:
        robot.stop(Stop.BRAKE)
        break

# robot stops for accuracy
robot.stop(Stop.BRAKE)

# robot beeps to indicate it completed the color sensor code
ev3.speaker.beep()

# robot turns right to set the left color sensor with the line follower
right_motor.run_angle(150, 510)

DRIVE_SPEED = 100

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2
runWhile = True
# Start following the line endlessly.
robot.reset()
while True:
    # Calculate the deviation from the threshold.
    deviation = left_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    print(robot.distance())
    if robot.distance() == 460:
        break

robot.stop(Stop.BRAKE)

ev3.speaker.beep()

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 80

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2
runWhile = True
# Start following the line endlessly.
while runWhile:
    # Calculate the deviation from the threshold.
    deviation = right_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # set the distance the robot needs to go
    print(right_sensor.reflection())
    if right_sensor.reflection() <= 13:
        runWhile = False

robot.stop(Stop.BRAKE)

ev3.speaker.beep()

front_largeMotor.run_angle(90, -150)

left_motor.run_angle(150, 110)

robot.straight(240)

robot.turn(-60)

robot.straight(50)

robot.turn(60)


'''
#follows the line 
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
runWhile = True
# Start following the line endlessly.
robot.reset()
while True:
    # Calculate the deviation from the threshold.
    deviation = left_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    print(robot.distance())
    if robot.distance() == 470:
        break

robot.stop(Stop.BRAKE)

ev3.speaker.beep()

robot.straight(5)
# the left color sensor senses the perpendicular black line to increase reliability
while True:
  
    robot.drive(100,0)
    print(right_sensor.reflection())
    if right_sensor.reflection() <= 10:
        robot.stop(Stop.BRAKE)
        break

# robot stops for accuracy
robot.stop(Stop.BRAKE)

# robot beeps to indicate it completed the color sensor code
ev3.speaker.beep()
'''