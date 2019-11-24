import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(8, GPIO.IN)
GPIO.setup(9, GPIO.IN)
GPIO.setup(10, GPIO.IN)
GPIO.setup(11, GPIO.IN)

outcome = [0,-1,1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0]
last_AB1 = 0b00
counter1 = 0
last_AB2 = 0b00
counter2 = 0


while True:
    A1 = GPIO.input(8)
    B1 = GPIO.input(9)
    current_AB1 = (A1 << 1) | B1
    position1 = (last_AB1 << 2) | current_AB1
    counter1 += outcome[position1]
    last_AB1 = current_AB1
    
    A2 = GPIO.input(10)
    B2 = GPIO.input(11)
    current_AB2 = (A2 << 1) | B2
    position2 = (last_AB2 << 2) | current_AB2
    counter2 += outcome[position2]
    last_AB2 = current_AB2
    
    print("Encoder 1:",counter1)
    #print("Encoder 2:",counter2)