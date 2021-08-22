#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S1)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.7, axle_track=111.2)

while True:
    corLvl = (line_sensor.reflection() - 25) * 2
    spdLvl = abs(100 - abs(25 - abs(line_sensor.reflection()))) 

    robot.drive(spdLvl, corLvl)