import RPi._GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

trigger = 10
echo = 8

GPIO.setup(trigger, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)

def distance():
    GPIO.output(trigger, True)
    time.sleep(0.1)

    GPIO.output(trigger, False)
    
    startTime = time.time()
    stopTime = time.time()

    while GPIO.input(echo) == 0:
        startTime = time.time()

    while GPIO.input(echo) == 1:
        stopTime = time.time()

    timeElapsed = stopTime - startTime
    distance = (timeElapsed * 34300) / 2

    return distance