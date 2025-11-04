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
# ลองตั้งความละเอียด (ปรับได้ตามกล้อง)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("เปิดกล้องไม่สำเร็จ")
    GPIO.cleanup()
    raise SystemExit

def led_blink(rounds):
    for _ in range(rounds):
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

cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)  # ย่อ/ขยายได้
cv2.resizeWindow("Camera", 960, 540)

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

try:
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
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_OPEN, kernel)
        mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_CLOSE, kernel)

        # หา contour
        contours_g, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_r, _ = cv2.findContours(mask_red,   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
                    # วาดกรอบ/จุด
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)

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
                    # วาดกรอบ/จุด
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

        GREEN_PIN_DETECTED = target_center_green is not None
        RED_PIN_DETECTED   = target_center_red is not None

        # สถานะบนจอ
        if GREEN_PIN_DETECTED and RED_PIN_DETECTED:
            status = "Green and Red pin detected!!!"
            color  = (0, 255, 255)
        elif RED_PIN_DETECTED:
            status = "Red pin detected!!!"
            color  = (0, 0, 255)
        else:
            status = "White pin detected."
            color  = (255, 255, 255)

        cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)

        # เส้นกึ่งกลาง (ช่วยเล็ง)
        h, w = frame.shape[:2]
        cv2.line(frame, (w//2, 0), (w//2, h), (200, 200, 200), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (200, 200, 200), 1)

        cv2.imshow("Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
