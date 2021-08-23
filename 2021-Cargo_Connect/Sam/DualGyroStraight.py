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

lDedAng = int(left_motor.angle())
rDedAng = int(right_motor.angle())

goalDisMil = 1000
goalDisDeg = (goalDisMil / wheelCirc) * 360

totAvgDeg = 0

while totAvgDeg < goalDisDeg:
    corLvl = ((gyro1.angle() + gyro2.angle()) / 2) * -0.6
    if totAvgDeg < goalDisDeg * 0.8:
        spdLvl = 1000
    elif totAvgDeg >= goalDisDeg * 0.8:
        spdLvl = 50

    robot.drive(spdLvl, corLvl)

    lAng = int(left_motor.angle() - lDedAng)
    rAng = int(right_motor.angle() - rDedAng)

    totAvgDeg = (lAng + rAng) / 2

robot.stop()