#Class to calculate PID controller
import time
import sys

class PIDController:
    def __init__(self, P, I, D, Time, lastI, lastError):
        self.KP = P
        self.KI = I
        self.KD = D
        self.time = Time
        self.target = 0

        self.lastError = lastError
        self.integrator = lastI

    def setTarget(self, newTarget):
        self.target = newTarget

    def step(self, currentValue):

        error = self.target - currentValue  #error = desired value - actual value

        output = ((self.KP * error) + (self.KI * self.integrator) + (self.KD * ((error - self.lastError)/self.time))) #PID equation

        self.lastError = error
        self.integrator += (error * self.time)

        return output, self.integrator, self.lastError
