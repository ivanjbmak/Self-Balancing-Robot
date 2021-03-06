# Necessary libraries for BNO055 IMU Sesnsor
import busio
import adafruit_bno055
from busio import I2C
from board import SDA, SCL
i2c = I2C(SCL, SDA)
import sys

#Necessary libraries and classes for RaspberryPi and PID
import time
import board
from time import sleep
import math
from pidcontroller import PIDController
import  RPi.GPIO as GPIO

#variables for time, I term and error term
last_time = float(0)
lastI = float(0)
lastError = 0

#Setting the Mode to use for RPi pins. We are using the BCM setup.
GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setwarnings(False)

#initilizing PWM for 4 pins
PWM1 = GPIO.PWM(12, 10000)
PWM2 = GPIO.PWM(13, 10000)
PWM3 = GPIO.PWM(18, 10000)
PWM4 = GPIO.PWM(19, 10000)

#function to drive motors forwards taking the PID value as the PWM
def forward(PWM):
    if PWM > 100: #Setting limit for PWM, MAX being 100 and MIN being 5 (PWM < 5 will not move motors)
        PWM = 100
    elif PWM < 5:
        PWM = 5
    PWM1.start(PWM)
    PWM3.start(PWM)
    GPIO.output(12, True)
    GPIO.output(13, True)
    GPIO.output(18, True)
    GPIO.output(19, False)

#function to drive motors backward taking the PID value as the PWM
def backward(PWM):
    if PWM > 100: #Setting limit for PWM, MAX being 100 and MIN being 5 (PWM < 5 will not move motors)
        PWM = 100
    elif PWM < 5:
        PWM = 5
    PWM1.start(PWM)
    PWM3.start(PWM)
    GPIO.output(12, True)
    GPIO.output(13, False)
    GPIO.output(18, True)
    GPIO.output(19, True)

#function to stop motors
def equilibrium():
    PWM1.start(0)
    PWM3.start(0)
    GPIO.output(12, False)
    GPIO.output(13, False)
    GPIO.output(18, False)
    GPIO.output(19, False)

#defining BNO055 IMU sensor
sensor = adafruit_bno055.BNO055(i2c)

#the so called 'main loop' that loops around and tells the motors wether to move or not
while True:

    gyro_data = sensor.euler #function to call raw gyroscope oriention data from IMU
    gyroY = (gyro_data[1])  #variable created to only grab the y-axis gyro data

    #variables for time for PID
    time1 = time.process_time()
    dt = time1-last_time
    last_time = time1

    #Calling PIDcontroller class from pidcontroller, setting P, I, and D gains
    PID = PIDController(P=10, I=0.05, D=0.09, Time = dt, lastI = lastI, lastError = lastError)
    PIDy, lastI, lastError = PID.step(gyroY)

    if gyroY < -50: #if tile angle is greater than 50, shut off motors (dead-band)
        equilibrium()

    elif gyroY > 50: #if tile angle is greater than 50, shut off motors (dead-band)
        equilibrium()

    elif PIDy < 0: #if PID value is less than 0, drive motors backwards
        backward(-PIDy)

    elif PIDy > 0: #if PID value is more than 0, drive motors forwards
        forward(PIDy)

    elif gyroY == 0: #if tilt angle is 0 (balanced), shut off motors.
        equilibrium()

    #print("Tilt Angle: ", float(gyroY))
    #print("PID: ", int(PIDy))
