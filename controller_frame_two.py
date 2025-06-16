import subprocess
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

MAX_RETRIES = 10
retry_count = 0

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
    while retry_count < MAX_RETRIES:
        try:
            result = subprocess.run(["python3", "frame_two.py"], check=True)
            print("Frame two script ran successfully.")
            break
        except subprocess.CalledProcessError as e:
            retry_count += 1
            led_blink(5)
            print(f"Frame two script failed (count: {retry_count}). Retrying ...")
    else:
        print("Failed 10 times. Running error_frame_two_three.py ...")
        subprocess.run(["python3", "error_frame_two_three.py"])
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    GPIO.cleanup()