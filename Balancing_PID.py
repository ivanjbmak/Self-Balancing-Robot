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


GPIO.setmode(GPIO.BCM) #Setting the Mode to use for RPi pins. We are using the BCM setup.
GPIO.setup(12, GPIO.OUT) #PWM1 Pin for motor 1
GPIO.setup(13, GPIO.OUT) #DIR1 pin for motor 1
GPIO.setup(18, GPIO.OUT) #PWM3 pin for motor 2
GPIO.setup(19, GPIO.OUT) #DIR2 pin for motor 2
GPIO.setwarnings(False)

#Creating PWM instances
PWM1 = GPIO.PWM(12, 100)
PWM2 = GPIO.PWM(13, 100)
PWM3 = GPIO.PWM(18, 100)
PWM4 = GPIO.PWM(19, 100)

#Function to call to move the robot forward
def forward(PWM):
    if PWM > 100: #PWM duty cycle value can only go from 0-100, if statement to set boundaries. 
        PWM = 100
    elif PWM < 5: #PWM of atleast 5, as any duty cycle lower than 5 won't move motors.
        PWM = 5
    PWM1.start(PWM)
    PWM3.start(PWM)
    GPIO.output(12, True) #Always true to keep motor 1 on
    GPIO.output(13, True) #Direction
    GPIO.output(18, True) #Always true to keep motor 2 on
    GPIO.output(19, False) #Direction

#Function to call to move the robot backward
def backward(PWM):
    if PWM > 100: #PWM duty cycle value can only go from 0-100, if statement to set boundaries.
        PWM = 100
    elif PWM < 5: #PWM of atleast 5, as any duty cycle lower than 5 won't move motors.
        PWM = 5
    PWM1.start(PWM)
    PWM3.start(PWM)
    GPIO.output(12, True) #Always true to keep motor 1 on
    GPIO.output(13, False) #Direction
    GPIO.output(18, True) #Always true to keep motor 2 on
    GPIO.output(19, True) #Direction

#Function to stop motors when robot is balanced at angle 0.
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

    #variable created to only grab the y-axis gyro data
    gyroY = gyro_data[2] + 8.265 # IMU sensor data is off by 8.265 degrees, this is to compensate for the angle.

    #setting the PID values. Here you can change the P, I and D values according to robot needs
    PID = PIDController(P=8, I=4, D=0.1)
    PIDy = PID.step(gyroY)

    #if the PIDy data is lower than 0.0 than move appropriately backward to balance
    if PIDy < 0:
        backward(-float(PIDy))
        #StepperFor(-PIDx)
    #if the PIDy data is higher than 0.0 than move appropriately forward to balance
    elif PIDy > 0:
        forward(float(PIDy))
        #StepperBACK(PIDx)
    elif PIDy == 0:
        equilibrium()


    #Constantly print out reading of gyro in the y axis and PIDy value
    print(float(gyroY))
    print(int(PIDy))
    sleep(0.02)

GPIO.cleanup()
