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

def BlackMission():
    
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
    
    robot.settings(140)

    robot.straight(350) 
    robot.straight(-97)
    robot.turn(-40)
  
    large_motor.run_angle(30,50,then=Stop.HOLD, wait=True)
    
    ev3.speaker.beep(15)
    large_motor.run_angle(60,-180,then=Stop.HOLD, wait=False)
    robot.straight(-300)
   
    
    wait(10000)
    robot.stop(Stop.BRAKE)
    robot.settings(240)
    ev3.speaker.beep(20)
    robot.straight(390)
    ev3.speaker.beep(300)
    large_motor.run_angle(60,130)
    
    robot.straight(-90)
    large_motor.run_angle(60,40,then=Stop.HOLD, wait=True)
    robot.straight(-500)
    # test straight after beep and see if it works...
    robot.stop(Stop.BRAKE)
    
def BlueMission():
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

    #go front towards the step counter
    robot.settings(250)
    robot.straight(650)
    robot.stop(Stop.BRAKE)
    wait(20)

    #makes the robot go slower 
    robot.settings(40)

    #slowly pushes the step counter by going back and front 2 times
    robot.straight(140)
    robot.stop(Stop.BRAKE)
    robot.straight(-45)
    robot.stop(Stop.BRAKE)
    robot.straight(120) 
    robot.stop(Stop.BRAKE)
    robot.straight(-30)
    robot.stop(Stop.BRAKE)
    #the robot then turns and goes backwards
    robot.settings(250,500,250,500)
    robot.turn(45)
    robot.straight(-100)
    # the robot then goes back until the right color sensor detects back
    while True:
        robot.drive(-115,0)
        if line_sensor.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break 
    #the large motor attatchment comes down at the same time the robot takes a turn towards the black line underneath the pull up bar
    large_motor.run_angle(50,200,then=Stop.HOLD, wait=False)
    left_motor.run_angle(50,-300,then=Stop.HOLD, wait=True)
    #the robot then goes straight towards that line
    robot.straight(120)
    robot.stop(Stop.BRAKE)
    #robot continues to go forwards until the left color sensor detects black
    while True:
        robot.drive(115,0)
        if line_sensor.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break 
    right_motor.run_angle(50,150,then=Stop.HOLD, wait=True)
    #the robot then turns with the right motor until it detects black
    while True:
        right_motor.run(85)
        if line_sensor.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break
    #follows the line underneath the pull up bar until the leftsensor detects black
        #follows the line underneath the pull up bar until the leftsensor detects black
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
    #the robot then turns towards the boccia aim and moves straight to push it towards the target and finishes the misison 
    robot.turn(25)
    robot.straight(250)
    robot.straight(-50)
    # this is importate kekekekee
    large_motor.run_angle(75,-65)
    robot.straight(-60)
    while True:
        robot.drive(-70,0)
        if line_sensor.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            ev3.speaker.beep()
            break 
    while True:
        left_motor.run(-50)
        if line_sensor1.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            ev3.speaker.beep()
            break
    left_motor.run_angle(50, -20)
    right_motor.run_angle(50, 20)
    robot.settings(200)
    robot.straight(60)
    robot.turn(-137)
    #this is also importante jekeke
    large_motor.run_angle(50,80)
    robot.straight(143)
    large_motor.run_angle(550, -120)
    robot.straight(-40)
    large_motor.run_angle(550, 120)
    robot.straight(40)
    large_motor.run_angle(550, -120)
    large_motor.run_angle(300, 30, then=Stop.HOLD, wait=True)
    #robot.straight(40)
    large_motor.run_angle(300, -100, then=Stop.HOLD, wait=True)
    #goes to collect the health unit near the basketball (goes back to base)
    robot.straight(-200)
    robot.turn(40)
    robot.straight(556)
    robot.straight(50)
    robot.stop(Stop.BRAKE)
    while True:
        left_motor.run(50)
        if line_sensor1.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            ev3.speaker.beep()
            break
    robot.stop(Stop.BRAKE)
    
    robot.reset()
    # Calculate the light threshold. Choose values based on your measurements.
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
    while runWhile:
        # Calculate the deviation from the threshold.
        deviation = line_sensor1.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

        if robot.distance() >= 210:
            runWhile = False
            break
    robot.stop(Stop.BRAKE)
    robot.turn(5.244312)
    robot.straight(700)
    #the robot then goes back until the right color sensor detects back
    '''
    while True:
        if line_sensor1.color() == Color.BLACK:
            robot.stop(Stop.BRAKE)
            break 
    
    #robot.straight(980)
    '''
    robot.stop(Stop.BRAKE) 
    
def GreenMission():

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

    ## Write your code here: 
    robot.settings(300)
    ## The robot goes straight until the Boccia Mission's target. 
    robot.straight(1050)

    ## The robot moves the large motor down to drop the cubes in the target. 
    front_largeMotor.run_angle(80, 110, then=Stop.HOLD, wait=True)

    ## BOCCIA SHARE !!!
    robot.straight(-200)
    robot.turn(-100)
    robot.straight(130)
    front_largeMotor.run_angle(-80, 105, then=Stop.HOLD, wait=True)
    robot.straight(-30)
    robot.turn(-100)
    robot.straight(-80)

    
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)
    robot.turn(-100)
    robot.turn(100)


    ## Dance Mission
    '''
    #The robot moves backwards to reach the Dance Floor so it can Dance as the last mission. 
    robot.straight(-185)
    robot.turn(-80)
    robot.straight(110) 


    ## The following code is all the dance moves we do for the Dance Mission. 

    robot.turn(-75)

    robot.straight(50)

    front_largeMotor.run_angle(60, 50)

    robot.straight(40)

    front_largeMotor.run_angle(60, -50)
    
    robot.straight(-40)

    front_largeMotor.run_angle(60, 50)

    robot.straight(+30)

    front_largeMotor.run_angle(60, -50)
    
    robot.straight(-45)

    robot.turn(-500)

    robot.turn(500)

    
    front_largeMotor.run_angle(60, 50)
    robot.straight(92)
    front_largeMotor.run_angle(60, -50)
    robot.straight(-20)
    robot.straight(35)
    front_largeMotor.run_angle(60, -50)
    robot.straight(-60)
    robot.turn(100)
    robot.turn(-80)
    robot.turn(120)
    robot.turn(-400)
    
    large_motor.run_angle(20, -10)
    robot.straight(-30)

    
    robot.turn(-160)
    robot.straight(60)
    robot.straight(-60)
    robot.turn(260)
    robot.turn(-260)
    robot.turn(100)
    robot.straight(40)
    robot.turn(100)
    robot.straight(-25)
    '''
    robot.stop(Stop.BRAKE)
    
def RedMission(): 
    
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

    robot.straight(200)
    robot.turn(-115)
    Medium_Motor.run_angle(300, 135, then=Stop.HOLD, wait=True)
    robot.stop(Stop.BRAKE)
    robot.settings(200)
    robot.turn(-60)
    robot.straight(400)

    Large_Motor.run_angle(80, 85, then=Stop.HOLD, wait=True)

    robot.stop(Stop.BRAKE)
    
waitButtons()

