#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import GyroSensor, Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from time import sleep

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

wheelDim = 56
wheelCirc = wheelDim * 3.14

axlTrk = 111

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter = wheelDim, axle_track=axlTrk)

gyro1Prt = Port.S2
gyro2Prt = Port.S3
gyro1 = GyroSensor(gyro1Prt)
gyro2 = GyroSensor(gyro2Prt)

gyro1.reset_angle(0)
gyro2.reset_angle(0)

turnDeg = 45

stopped = False

if turnDeg > 0:
    while (gyro1.angle() + gyro2.angle()) / 2 < turnDeg - 0.5:
        if (gyro1.angle() + gyro2.angle()) / 2 < 0.5 * turnDeg:
            left_motor.run(200)
            right_motor.run(-200)
        elif (gyro1.angle() + gyro2.angle()) / 2 >= 0.5 * turnDeg:
            if stopped == False:
                left_motor.stop()
                right_motor.stop()
                stopped = True
            left_motor.run(25)
            right_motor.run(-25)
    left_motor.hold()
    right_motor.hold()    

elif turnDeg < 0:
    while (gyro1.angle() + gyro2.angle()) / 2 > turnDeg + 0.5:
        if (gyro1.angle() + gyro2.angle()) / 2 > 0.5 * turnDeg:
            left_motor.run(-200)
            right_motor.run(200)
        elif (gyro1.angle() + gyro2.angle()) / 2 <= 0.5 * turnDeg:
            if stopped == False:
                left_motor.stop()
                right_motor.stop()
                stopped = True
            left_motor.run(-25)
            right_motor.run(25)
    left_motor.hold()
    right_motor.hold()