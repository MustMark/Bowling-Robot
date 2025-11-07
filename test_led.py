import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

try:
    while True:
        GPIO.output(7, GPIO.LOW)
        GPIO.output(8, GPIO.HIGH)
        GPIO.output(24, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(7, GPIO.HIGH)
        GPIO.output(8, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(7, GPIO.HIGH)
        GPIO.output(8, GPIO.HIGH)
        GPIO.output(24, GPIO.LOW)
        time.sleep(0.2)
except KeyboardInterrupt:
    GPIO.cleanup()
