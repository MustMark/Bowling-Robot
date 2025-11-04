import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import subprocess

GPIO.setmode(GPIO.BCM)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

MAX_RETRIES = 30
retry_count = 0

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("เปิดกล้องไม่สำเร็จ")
    exit()


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

tolerance = 20 
GREEN_PIN_DETECTED = False
RED_PIN_DETECTED = False

while True:
    ret, frame = cap.read()
    if not ret:
        print("ไม่สามารถอ่านกล้องได้")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # เขียว
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # แดง (สองช่วง)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

    # ลบ noise
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

    # หา contour ของแต่ละสี แยกกัน
    contours_g, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_r, _ = cv2.findContours(mask_red,   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # เลือกคอนทัวร์ที่ใหญ่สุดของแต่ละสี
    target_center_green = None
    target_center_red = None

    max_area_g = 0
    for cnt in contours_g:
        area = cv2.contourArea(cnt)
        if area > 300 and area > max_area_g:
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                target_center_green = (cx, cy)
                max_area_g = area

    max_area_r = 0
    for cnt in contours_r:
        area = cv2.contourArea(cnt)
        if area > 300 and area > max_area_r:
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                target_center_red = (cx, cy)
                max_area_r = area

    GREEN_PIN_DETECTED = target_center_green is not None
    RED_PIN_DETECTED   = target_center_red is not None

    if GREEN_PIN_DETECTED and RED_PIN_DETECTED:
        print("Green and Red pin detected!!!")
        # try:
        #     while retry_count < MAX_RETRIES:
        #         try:
        #             result = subprocess.run(["python3", "/home/ubuntu/Desktop/frame_two.py"], check=True)
        #             print("Frame one script ran successfully.")
        #             break
        #         except subprocess.CalledProcessError as e:
        #             retry_count += 1
        #             led_blink(5)
        #             print(f"Frame one script failed (count: {retry_count}). Retrying ...")
        #     else:
        #         print("Failed 10 times. Running error_frame_one.py ...")
        #         subprocess.run(["python3", "/home/ubuntu/Desktop/error_frame_one.py"])
        # except KeyboardInterrupt:
        #     print("Stopped by user.")
        # finally:
        #     GPIO.cleanup()
    elif RED_PIN_DETECTED:
        print("Red pin detected!!!")
        # try:
        #     while retry_count < MAX_RETRIES:
        #         try:
        #             result = subprocess.run(["python3", "/home/ubuntu/Desktop/frame_three.py"], check=True)
        #             print("Frame one script ran successfully.")
        #             break
        #         except subprocess.CalledProcessError as e:
        #             retry_count += 1
        #             led_blink(5)
        #             print(f"Frame one script failed (count: {retry_count}). Retrying ...")
        #     else:
        #         print("Failed 10 times. Running error_frame_one.py ...")
        #         subprocess.run(["python3", "/home/ubuntu/Desktop/error_frame_one.py"])
        # except KeyboardInterrupt:
        #     print("Stopped by user.")
        # finally:
        #     GPIO.cleanup()
    else:
        print("White pin detected.")
        # try:
        #     while retry_count < MAX_RETRIES:
        #         try:
        #             result = subprocess.run(["python3", "/home/ubuntu/Desktop/frame_one.py"], check=True)
        #             print("Frame one script ran successfully.")
        #             break
        #         except subprocess.CalledProcessError as e:
        #             retry_count += 1
        #             led_blink(5)
        #             print(f"Frame one script failed (count: {retry_count}). Retrying ...")
        #     else:
        #         print("Failed 10 times. Running error_frame_one.py ...")
        #         subprocess.run(["python3", "/home/ubuntu/Desktop/error_frame_one.py"])
        # except KeyboardInterrupt:
        #     print("Stopped by user.")
        # finally:
        #     GPIO.cleanup()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
