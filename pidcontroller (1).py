'''HARIOMHARIBOLJAIMATAJIPITAJIKIJAIJAI'''
#Developed by Mark Tomczak (fixermark@gmail.com)
#Python class to calculate PID values
import time
import sys



class PIDController:
    def __init__(self, P, I, D, S, lastI, lastError):
        self.KP = P
        self.KI = I
        self.KD = D
        self.time = S
        self.target = 0

        self.lastError = lastError
        self.integrator = lastI

    def setTarget(self, newTarget):
        self.target = newTarget
        

    def step(self, currentValue):
        
        error = currentValue - self.target

        output = (self.KP * error + self.KI * self.integrator + self.KD * ((error - self.lastError)/self.time))

        self.lastError = error
        self.integrator += error * self.time

        
        return output, self.integrator, self.lastError
    

    