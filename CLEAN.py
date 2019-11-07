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

last_time = float(0)
lastI = float(0)
lastError = 0
direction = 2
lastIAngle = 0


GPIO.setmode(GPIO.BCM) #Setting the Mode to use for RPi pins. We are using the BCM setup.
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setwarnings(False)

PWM1 = GPIO.PWM(12, 100)
PWM2 = GPIO.PWM(13, 100)
PWM3 = GPIO.PWM(18, 100)
PWM4 = GPIO.PWM(19, 100)

#pwm = 50

def forward(pwm):
    if pwm > 100:
        pwm = 100
    elif pwm < 5:
        pwm = 5
    PWM1.start(pwm)
    PWM3.start(pwm)
    GPIO.output(12, True)
    GPIO.output(13, True)
    GPIO.output(18, True)
    GPIO.output(19, False)
  
def backward(pwm):
    if pwm > 100:
        pwm = 100
    elif pwm < 5:
        pwm = 5
    PWM1.start(pwm)
    PWM3.start(pwm)
    GPIO.output(12, True)
    GPIO.output(13, False)
    GPIO.output(18, True)
    GPIO.output(19, True)
    
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
    gyroX = (gyro_data[2]) + 8  #variable created to only grab the y-axis gyro data

    #setting the PID values. He1e you can change the P, I and D values according to our robot needs
    time1 = time.process_time()
    
    dt = time1-last_time
    
    last_time = time1
    
    PID = PIDController(P=6, I=30, D=0.08 ,S = dt, lastI = lastI, lastError = lastError)
            
    PIDx, lastI, lastError = PID.step(gyroX)
    #print(lastI)

    #if the PIDx data is lower than 0.0 than move appropriately backward
    if PIDx < 0:
        backward(-int(PIDx))
       # if direction == 0:
           # lastI = 0
        
      #  direction  = 1
        
        
        
        #StepperFor(-PIDx)
    #if the PIDx data is higher than 0.0 than move appropriately forward
    elif PIDx > 0:
        forward(int(PIDx))
       # if direction == 1:
         #   lastI = 0
        
      #  direction = 0
        
        #StepperBACK(PIDx)
    elif gyroX == 0:
        equilibrium()
        
        

    #Constantly print out reading of gyro in the y axis.
    print(float(gyroX))
    #print(int(PIDx))
    #print(lastI)
#     print(time1)
    wait(0.0005)


