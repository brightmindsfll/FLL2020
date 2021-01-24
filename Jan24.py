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


# Buttons Function - To make it easier to start each program. 

def waitButtons():
    while True:
        ev3.light.on(Color.GREEN)
        if Button.UP in ev3.buttons.pressed(): 
            RedMission()
        if Button.LEFT in ev3.buttons.pressed():
            BlackMission()
        if Button.DOWN in ev3.buttons.pressed():
            BlueMission()
        if Button.RIGHT in ev3.buttons.pressed():
            GreenMission()

def BlackMission(): # Black Run (Innovatice Architecture, Health Units, Hopscotch, Bringing Slide Figures back HOME)
    # LEFT BUTTON
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

    # define your variables
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
    
    robot.settings(500) # To change the SPEED

    # Pushing our innovative architecture and the health units.
    robot.straight(-350) 
    robot.straight(50)
    robot.turn(-15)
    robot.straight(-70)
    robot.turn(-220)
    robot.stop(Stop.BRAKE)
    robot.straight(-340)
    robot.stop(Stop.BRAKE)

    
def BlueMission(): # Blue Run (Step Counter, Pull-Up Bar, Boccia Aim, Slide, Health Unit - 1)
    # DOWN BUTTON
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
  
    # Go towards the step counter mission from base
    robot.settings(800) # Speed Change
    robot.straight(650)
    robot.stop(Stop.BRAKE)
    wait(20)

    # Slow the robot down to succesfully push the step counter. 
    robot.settings(200)

    # Slowly pushes the step counter by going backward and forward a couple times to increase reliability. 
    robot.straight(230)
    robot.straight(-20)
    robot.straight(50)
    robot.stop(Stop.BRAKE)
    
    #robot.straight(-45)
    #robot.stop(Stop.BRAKE)
    #robot.straight(120) 
    #robot.stop(Stop.BRAKE)
    
    robot.straight(-60)
    robot.stop(Stop.BRAKE)
    
    # The robot then turns and goes backwards until the right color sensor detects black. 
    #robot.settings(250,300,250,300)
    robot.turn(45)
    robot.straight(-100)

    while True:
        robot.drive(-100,0)
        if line_sensor.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break 

    #The large motor attatchment comes down at the same time the robot takes a turn towards 
    #the black line underneath the pull up bar
    left_motor.run_angle(50,-300,then=Stop.HOLD, wait=True)
    
    # The robot then goes straight towards the line under the pull-up bar. 
    robot.straight(120)
    robot.stop(Stop.BRAKE)
    
    # Robot continues to go forwards until the left color sensor detects black.
    while True:
        robot.drive(115,0)
        if line_sensor.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break 
    right_motor.run_angle(100,150,then=Stop.HOLD, wait=True)
    
    # The robot turns using the right motor until it detects black.
    while True:
        right_motor.run(100)
        if line_sensor.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break
    robot.straight(-90)
    large_motor.run_angle(100,150,then=Stop.HOLD, wait=True)

    robot.stop(Stop.BRAKE)
    
    robot.stop(Stop.BRAKE)

    while True:
        right_motor.run(40)
        if line_sensor.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break

    robot.stop(Stop.BRAKE)

    ev3.speaker.beep()
    
    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2
    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 100
    # Set the gain of the proportional line controller. This means that for every
    PROPORTIONAL_GAIN = 1.2
    runWhile = True
    robot.reset()
    ev3.speaker.beep()
    while True:
        # Calculate the deviation from the threshold.
        deviation = line_sensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation
        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)
        print(line_sensor1.color())
        if line_sensor1.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break
    robot.stop(Stop.BRAKE)

    
    robot.stop(Stop.BRAKE)
    large_motor.run_angle(-150, 150, then=Stop.HOLD, wait=False)
    robot.turn(20)
    robot.stop(Stop.BRAKE)
    robot.settings(800)
    robot.straight(280)
    ev3.speaker.beep(3)
    while True:
        robot.drive(-115,0)
        if line_sensor.color() == Color.BLACK:
            ev3.speaker.beep(10)
            robot.stop(Stop.BRAKE)
            break 
    robot.stop(Stop.BRAKE)
    # robot.straight(-10)
    robot.stop(Stop.BRAKE)
    robot.turn(50)
    
    # left_motor.run_angle(100, 150)
    '''
    large_motor.run_angle(30,-20,then=Stop.HOLD, wait=False)
    robot.turn(10)
    robot.stop(Stop.BRAKE)
    
    large_motor.run_angle(100, -50, then=Stop.HOLD, wait=False)
    robot.turn(90)
    robot.stop(Stop.BRAKE)
    
    '''
    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2
    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 100
    # Set the gain of the proportional line controller. This means that for every
    PROPORTIONAL_GAIN = 1.2
    runWhile = True
    robot.reset()
    ev3.speaker.beep()
    while True:
        # Calculate the deviation from the threshold.
        deviation = line_sensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation
        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)
        print(line_sensor.color())
        if robot.distance() >= 500:
            robot.stop(Stop.BRAKE)
            break
    ev3.speaker.beep(3)

    while True:
        robot.drive(40,0)
        if line_sensor1.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break

    robot.stop(Stop.BRAKE)
    ev3.speaker.beep()
    robot.straight(30)
    robot.turn(-100)
    robot.straight(70)
    large_motor.run_angle(600,150,then=Stop.HOLD, wait=True)

    while True:
        robot.drive(-50, 0)
        if line_sensor1.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break
    robot.stop(Stop.BRAKE)

    right_motor.run_angle(600,500,then=Stop.HOLD,wait=True)

    
    robot.straight(20)
    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2
    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 100
    # Set the gain of the proportional line controller. This means that for every
    PROPORTIONAL_GAIN = 1.2
    runWhile = True
    robot.reset()
    ev3.speaker.beep()
    while True:
        # Calculate the deviation from the threshold.
        deviation = line_sensor1.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation
        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)
        print(line_sensor1.color())
        if robot.distance() >= 580:
            robot.stop(Stop.BRAKE)
            break
    robot.stop(Stop.BRAKE)
    
    while True:
        robot.drive(50, 0)
        if line_sensor.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break
    robot.stop(Stop.BRAKE)
    ev3.speaker.beep(3)

    robot.turn(-45)
    robot.stop(Stop.BRAKE)
    
    robot.straight(30)
    
    large_motor.run_angle(1000,-150,then=Stop.HOLD, wait=True)
    robot.straight(-40)
    large_motor.run_angle(1000,150,then=Stop.HOLD, wait=True)
    robot.straight(40)
    large_motor.run_angle(1000,-150,then=Stop.HOLD, wait=True)

    robot.straight(-115)
    robot.turn(95)
    robot.straight(420)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    

def GreenMission(): # Green Run (Boccia Frame, Boccia Share, and Dance Mission)
    # RIGHT BUTTON
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
    axle_track = 114.3 

    robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
    robot.settings(300) 
    robot.straight(1050) 
    front_largeMotor.run_angle(80, 110, then=Stop.HOLD, wait=True)
    front_largeMotor.run_angle(80, -110, then=Stop.HOLD, wait=False)
    robot.straight(-1100)
    wait(7000)
    robot.straight(380)
    front_largeMotor.run_angle(60,90)
    robot.straight(-150)
    front_largeMotor.run_angle(60,40,then=Stop.HOLD, wait=True)
    robot.straight(-500)
    front_largeMotor.run_angle(60,-130,then=Stop.HOLD, wait=True)
    ev3.speaker.beep(7000)
    robot.stop(Stop.BRAKE)
 
def RedMission(): # Red Run (Bench Mission (including backrest removal))
    # UP BUTTON
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
    wheel_diameter = 56
    axle_track = 115

    robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

    Medium_Motor = Motor(Port.A)
    Large_Motor = Motor(Port.D)

    leftcolorsensor = ColorSensor(Port.S3)
    rightcolorsensor = ColorSensor(Port.S2)

    robot.settings(500)
    robot.straight(290)
    robot.stop(Stop.BRAKE)
    robot.settings(700,400,700,400)
    robot.turn(110)
    robot.stop(Stop.BRAKE)
    while True:
        robot.drive(200,0)
        if leftcolorsensor.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break 
    robot.stop(Stop.BRAKE)
    ev3.speaker.beep(3)
    robot.turn(-110)
    robot.straight(80)
    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2
    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 100
    # Set the gain of the proportional line controller. This means that for every
    PROPORTIONAL_GAIN = 1.2
    runWhile = True
    robot.reset()
    ev3.speaker.beep()
    
    while True:
        # Calculate the deviation from the threshold.
        deviation = rightcolorsensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation
        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)
        print(rightcolorsensor.color())
        
        if robot.distance() >= 100:
            robot.stop(Stop.BRAKE)
            break
    robot.stop(Stop.BRAKE)
    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2
    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 100
    # Set the gain of the proportional line controller. This means that for every
    PROPORTIONAL_GAIN = 1.2
    runWhile = True
    robot.reset()
    
    while True:
        # Calculate the deviation from the threshold.
        deviation = rightcolorsensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation
        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)
        print(rightcolorsensor.color())
        
        if leftcolorsensor.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break

    robot.turn(-25)
    robot.straight(230)
    Large_Motor.run_angle(50,100,then = Stop.HOLD, wait = True)
    
    robot.straight(-60)
    robot.turn(35)
    robot.straight(-10)
    Large_Motor.run_angle(50,-50,then = Stop.HOLD, wait = True)
    robot.straight(-85)
    robot.turn(-85)
    robot.straight(500)
    robot.turn(-20)
    robot.straight(250)
    Large_Motor.run_angle(50,-70,then = Stop.HOLD, wait = False)
    robot.turn(110)
    robot.stop(Stop.BRAKE)
    
# This is the part of the code that calls the function so we can actually USE our buttons code. 
waitButtons()

