#!/usr/bin/python3
# coding=utf8
import sys
import rospy
import signal
import time
import RPi.GPIO as GPIO
from threading import Thread
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_msgs.msg import RawIdPosDur
from chassis_control.msg import *
import smbus
import cv2
import numpy as np
import subprocess

GPIO.setmode(GPIO.BCM)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

MAX_RETRIES = 30
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

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

directions = {
    'forward': 90,
    'backward': 270,
    'left': 180,
    'right': 0,
    'forward_left': 135,
    'forward_right': 45,
    'backward_left': 225,
    'backward_right': 315
}

def run_with_retry(script_path, max_retries=MAX_RETRIES):
    """รันสคริปต์ด้วย subprocess พร้อมรีทรายและกระพริบ LED ถ้าล้มเหลว"""
    retry_count = 0
    try:
        while retry_count < max_retries:
            try:
                result = subprocess.run(["python3", script_path], check=True)
                print(f"{script_path} ran successfully.")
                return True
            except subprocess.CalledProcessError as e:
                retry_count += 1
                led_blink(5)
                print(f"{script_path} failed (count: {retry_count}). Retrying ...")
        else:
            print(f"Failed {max_retries} times. Running error_frame_one.py ...")
            subprocess.run(["python3", "/home/ubuntu/Desktop/error_frame_one.py"])
            return False
    except KeyboardInterrupt:
        print("Stopped by user.")
        return False

def move(direction, distance, speed, yaw_speed):
    duration = distance / speed
    set_velocity.publish(speed, direction, 0)
    start_time = time.time()
    while time.time() - start_time < duration:
        rospy.sleep(0.05)

def rotate(direction, duration):
    if direction == "right":
        set_velocity.publish(0, 0, 0.04)
    else:
        set_velocity.publish(0, 0, -0.04)
    rospy.sleep(duration)

def calibrate():
    while True:
        sensor_data = read_sensor_i2c()
        IR_1 = sensor_data[1]
        IR_4 = sensor_data[4]

        print(f"IR_1: {IR_1}, IR_4: {IR_4}")

        if IR_4 == '1' and IR_1 == '1':
            move(directions['forward'], 1, 20, 0)

        elif IR_4 == '1' and IR_1 == '0':
            rotate("left", 0.0001)

        elif IR_4 == '0' and IR_1 == '1':
            rotate("right", 0.0001)

        elif IR_4 == '0' and IR_1 == '0':
            break

def set_servos(pub, duration, pos_s):
    msg = MultiRawIdPosDur(id_pos_dur_list=[
        RawIdPosDur(int(x[0]), int(x[1]), int(duration)) for x in pos_s])
    pub.publish(msg)

def catch_ball():
    set_servos(joints_pub, 800, ((1,0), (2,0), (3,200), (4,700), (5,300), (6,500)))
    rospy.sleep(1)
    set_servos(joints_pub, 800, ((1,0), (2,900), (3,200), (4,700), (5,300), (6,930)))
    rospy.sleep(1)
    set_servos(joints_pub, 600, ((1,0), (2,900), (3,270), (4,630), (5,100), (6,930)))
    rospy.sleep(5)
    set_servos(joints_pub, 600, ((1,800), (2,900), (3,270), (4,630), (5,100), (6,930)))
    rospy.sleep(2)
    set_servos(joints_pub, 600, ((1,800), (2,900), (3,270), (4,630), (5,330), (6,930)))
    rospy.sleep(2)
    set_servos(joints_pub, 600, ((1,800), (2,900), (3,270), (4,630), (5,330), (6,440)))
    rospy.sleep(2)
    set_servos(joints_pub, 600, ((1,800), (2,500), (3,250), (4,700), (5,330), (6,460)))
    rospy.sleep(2)

def release_ball():
    set_servos(joints_pub, 800, ((1,0), (2,500), (3,250), (4,700), (5,330), (6,460)))
    rospy.sleep(1)

def stop(time = 0):
    set_velocity.publish(0, 0, 0)
    rospy.sleep(time)

def shutdown(signum, frame):
    rospy.loginfo('shutdown')
    GPIO.cleanup()
    rospy.signal_shutdown('shutdown')

signal.signal(signal.SIGINT, shutdown)

bus = smbus.SMBus(1)
ARDUINO_ADDR = 0x08
MAX_BYTES = 32

def read_sensor_i2c():
    try:
        data = bus.read_i2c_block_data(ARDUINO_ADDR, 0, MAX_BYTES)
        ir_state = bytearray(data).split(b'\x00')[0].decode('utf-8', errors='ignore')
        return ir_state
    except Exception as e:
        print("Error reading from Arduino:", e)
        return None


# ====== เพิ่มฟังก์ชันตรวจจับสี + ฟังก์ชันกวาดทางขวา ======
def detect_colors(frame):
    """
    คืนค่าชุดสีที่พบในเฟรม: {'red', 'green', 'white'}
    ปรับ HSV ให้ทนแสงจริงหน้างาน (S,V ต่ำลงเล็กน้อย)
    """
    colors = set()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # white: ค่า S ต่ำ V สูง
    lower_white = np.array([0, 0, 180])
    upper_white = np.array([180, 60, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

    # green
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    lower_red1 = np.array([0, 120, 120])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 120])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red1, upper_red1),
        cv2.inRange(hsv, lower_red2, upper_red2)
    )

    # ทำความสะอาดมาสก์
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
    mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
    mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_OPEN, kernel)
    mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_CLOSE, kernel)

    # ฟังก์ชันกรอง blob ใหญ่จริง ไม่เอาแถบแดงแคบ
    def has_big_red_blob(mask, min_area=500):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = h / float(w + 1e-6)
            fill_ratio = area / float(w * h)
            # ต้องเป็นก้อนค่อนข้างสูงและหนาแน่นพอ
            if aspect > 0.8 and fill_ratio > 0.4:
                return True
        return False

    def has_big_blob(mask, min_area=300):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return any(cv2.contourArea(cnt) >= min_area for cnt in contours)

    if has_big_red_blob(mask_red):
        colors.add('red')
    if has_big_blob(mask_green):
        colors.add('green')
    if has_big_blob(mask_white):
        colors.add('white')

    return colors



def cv4_or(a, b):
    """bitwise OR รองรับ dtype ทั้งคู่แบบปลอดภัย"""
    return cv2.bitwise_or(a, b)

def sweep_right_and_summarize(cap, steps=40, step_dist=2, speed=40, pause=0.25, show_preview=False):

    seen = set()  # สีกลางทริปทั้งหมดที่เคยพบ

    # อุ่นกล้องนิดหน่อย
    for _ in range(5):
        cap.read()
        time.sleep(0.05)

    for i in range(steps):
        # อ่านเฟรมก่อนขยับ (หรือจะอ่านหลังขยับก็ได้)
        ret, frame = cap.read()
        if ret:
            seen |= detect_colors(frame)

        # ขยับไปทางขวา
        move(directions['right'], step_dist, speed, 0)
        stop(0)  # หยุด zero turn (ใช้ของคุณอยู่แล้ว)

        # พักสั้นๆ ให้กล้องนิ่ง/AE ทำงาน
        time.sleep(pause)

        # (ตัวเลือก) ดูพรีวิวระหว่างกวาด แต่ "ไม่พิมพ์สรุป" จนกว่าจะจบ
        if show_preview and ret:
            preview = frame.copy()
            txt = f"SWEEP {i+1}/{steps}  seen: {', '.join(sorted(list(seen))) if seen else '-'}"
            cv2.putText(preview, txt, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 255, 200), 2)
            cv2.imshow("Sweep Preview", preview)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # จบแล้วค่อยพิมพ์สรุปครั้งเดียว
    label_map_th = {'red': 'แดง', 'green': 'เขียว', 'white': 'ขาว'}
    if seen:
        summary_th = ", ".join(label_map_th[c] for c in sorted(seen))
        print(f"[SUMMARY] พบสี: {summary_th}")
    else:
        print("[SUMMARY] ไม่พบสีที่สนใจ")

    # ปิดหน้าต่างพรีวิวถ้ามี
    if show_preview:
        cv2.destroyWindow("Sweep Preview")
    
    cap.release()
    cv2.destroyAllWindows()

    return seen
# ====== จบส่วนเพิ่ม ======

if __name__ == '__main__':
    rospy.init_node('bid_team_node', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)

    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    joints_pub   = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(1)

    GPIO.output(8, GPIO.LOW)
    print("Press Start Button !")
            
    while True:
        sensor_data = read_sensor_i2c()

        if sensor_data[7] == '0':
            GPIO.output(8, GPIO.HIGH)
            break

    release_ball()

    stop(0.2)

    calibrate()

    stop(0.2)

    move(directions['backward'], 20, 50, 0)

    stop(0.2)

    calibrate()

    stop(0.2)

    cap = None
    try:
        # เปิดกล้องแบบประหยัดแบนด์วิธ (สำคัญมากบน Pi)
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 15)

        if not cap.isOpened():
            print("cannot open camera")
            sys.exit(1)

        # ---- ตัวอย่างการกวาดทางขวาแล้วสรุปสี ----
        # ปรับพารามิเตอร์ให้เข้ากับสปีด/หน่วย distance ของโรบอตคุณ
        seen_colors = sweep_right_and_summarize(
            cap,
            steps=55,        # จำนวนสเต็ปที่กวาด (เพิ่มถ้าต้องกวาดไกล)
            step_dist=2,     # ระยะต่อสเต็ป (ต้องสัมพันธ์กับ speed ใน move)
            speed=40,        # ความเร็วเคลื่อนที่ของ move()
            pause=0.25,      # เว้นให้ภาพนิ่งก่อนอ่านเฟรมถัดไป
            show_preview=True  # True ถ้าอยากดูพรีวิวระหว่างกวาด
        )


        if 'red' in seen_colors and 'green' in seen_colors:
            # สีแดงและเขียว
            run_with_retry("/home/ubuntu/Desktop/frame_two.py")
            GPIO.output(7, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(7, GPIO.LOW)
            GPIO.cleanup()
        elif 'red' in seen_colors:
            # สีแดงอย่างเดียว
            run_with_retry("/home/ubuntu/Desktop/frame_three_fix.py")
            GPIO.output(8, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(8, GPIO.LOW)
            GPIO.cleanup()
        else:
            # นอกจากนั้น (เช่น ขาว หรือไม่เจออะไร)
            run_with_retry("/home/ubuntu/Desktop/frame_one.py")
            GPIO.output(24, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(24, GPIO.LOW)
            GPIO.cleanup()


    except KeyboardInterrupt:
        pass
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        stop(0)
        GPIO.cleanup()
        print("PROGRAM DONE")
