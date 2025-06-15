import subprocess
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

def led_blink(round):
    for i in range(round):
        GPIO.output(7, GPIO.LOW)
        GPIO.output(8, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(7, GPIO.HIGH)
        GPIO.output(8, GPIO.HIGH)
        GPIO.output(24, GPIO.HIGH)
        time.sleep(0.1)

try:
    while True:
        try:
            result = subprocess.run(["python3", "frame_one.py"], check=True)
            print("Frame one script ran successfully.")
            break
        except subprocess.CalledProcessError as e:
            led_blink(5)
            print("Frame one script failed. Retrying ...")
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    GPIO.cleanup()