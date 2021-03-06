#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

sensePort = Port.S1

# Initialize the color sensor.
line_sensor = ColorSensor(sensePort)

wheelDim = 56
wheelCirc = wheelDim * 3.14

axlTrk = 111

side = "L"
sideNum = 0
if side.lower().startswith("l"):
    sideNum = 1
elif side.lower().startswith("r"):
    sideNum = -1

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter = wheelDim, axle_track=axlTrk)

goalDisMil = 1000
goalDisDeg = (goalDisMil / wheelCirc) * 360

lDedAng = int(left_motor.angle())
rDedAng = int(right_motor.angle())

totAvgDeg = 0

while totAvgDeg < goalDisDeg:

    corLvl = (line_sensor.reflection() - 25) * 2 * sideNum
    spdLvl = abs(100 - abs(25 - abs(line_sensor.reflection()))) 

    robot.drive(spdLvl, corLvl)

    lAng = int(left_motor.angle() - lDedAng)
    rAng = int(right_motor.angle() - rDedAng)

    totAvgDeg = (lAng + rAng) / 2

robot.stop()