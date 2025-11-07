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

GPIO.setmode(GPIO.BCM)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

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
    aligned_count = 0
    aligned_threshold = 3

    while True:
        sensor_data = read_sensor_i2c()
        IR_1 = sensor_data[1]
        IR_4 = sensor_data[4]

        print(f"IR_1: {IR_1}, IR_4: {IR_4}")

        if IR_4 == '0' and IR_1 == '0':
            aligned_count += 1
            print(f"[Aligned Check] {aligned_count}/{aligned_threshold}")
            move(directions['forward'], 1, 20, 0)

            if aligned_count >= aligned_threshold:
                print("[INFO] Aligned on black line. Stop.")
                stop()
                break

        else:
            aligned_count = 0 

            if IR_4 == '1' and IR_1 == '0':
                print("Adjusting left (Right is white)")
                rotate("left", 0.0001)

            elif IR_4 == '0' and IR_1 == '1':
                print("Adjusting right (Left is white)")
                rotate("right", 0.0001)

            elif IR_4 == '1' and IR_1 == '1':
                print("[INFO] Both white (no line yet), moving forward slowly...")
                move(directions['forward'], 1, 20, 0)


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

def detect_colors(frame):
    colors = set()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 180])
    upper_white = np.array([180, 60, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

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

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
    mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
    mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_OPEN, kernel)
    mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_CLOSE, kernel)

    def has_big_red_blob(mask, min_area=500):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = h / float(w + 1e-6)
            fill_ratio = area / float(w * h)
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
    return cv2.bitwise_or(a, b)

def sweep_right_and_summarize(cap, steps=40, step_dist=2, speed=40, pause=0.25, show_preview=False):

    seen = set()

    for _ in range(5):
        cap.read()
        time.sleep(0.05)

    for i in range(steps):
        ret, frame = cap.read()
        if ret:
            seen |= detect_colors(frame)

        move(directions['right'], step_dist, speed, 0)
        stop(0)

        time.sleep(pause)

        if show_preview and ret:
            preview = frame.copy()
            txt = f"SWEEP {i+1}/{steps}  seen: {', '.join(sorted(list(seen))) if seen else '-'}"
            cv2.putText(preview, txt, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 255, 200), 2)
            cv2.imshow("Sweep Preview", preview)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    if seen:
        print(f"[SUMMARY] : {seen}")
    else:
        print("[SUMMARY] : Not Found")

    if show_preview:
        cv2.destroyWindow("Sweep Preview")
    
    cap.release()
    cv2.destroyAllWindows()

    return seen

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


if __name__ == '__main__':

    rospy.init_node('bid_team_node', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)

    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(1)

    first_round = True
    red_pin_found_first_round = False

    for ball_round in range(3):

        first_round = (ball_round == 0)
        required_confirmations = 3
        confirm_count = 0
        tolerance = 10
        min_pin_count = 2
        max_seen_pins = 0
        mode = "scan"
        found_white_pin = False
        white_pin_detected_once = False
        white_pin_disappeared_after_detected = False
        red_pin_found = False
        found_left_black = False

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("cannot open camera")
            sys.exit(1)

        if ball_round == 0:
                while True:
                    move(directions['right'], 1, 40, 0)

                    sensor_data = read_sensor_i2c()

                    print(sensor_data)

                    IR_RIGHT = sensor_data[0]
                    print("IR_RIGHT:", IR_RIGHT)

                    if IR_RIGHT == '0':
                        confirm_count += 1
                    else:
                        confirm_count = 0

                    if confirm_count >= required_confirmations:
                        break
        elif ball_round == 1:
                
                release_ball()

                stop(0.2)

                calibrate()

                stop(0.2)

                move(directions['backward'], 20, 30, 0)

                stop(0.2)

                calibrate()

                stop(0.2)

                move(directions['right'], 400, 90, 0)

                while True:
                    move(directions['right'], 1, 40, 0)

                    sensor_data = read_sensor_i2c()

                    print(sensor_data)

                    IR_RIGHT = sensor_data[0]
                    print("IR_RIGHT:", IR_RIGHT)

                    if IR_RIGHT == '0':
                        confirm_count += 1
                    else:
                        confirm_count = 0

                    if confirm_count >= required_confirmations:
                        break
        elif ball_round == 2:

                    release_ball()

                    stop(0.2)

                    calibrate()

                    stop(0.2)

                    move(directions['backward'], 20, 30, 0)

                    stop(0.2)

                    calibrate()

                    stop(0.2)

                    print("[INFO] Warming up camera...")
                    for i in range(20):
                        ret, frame = cap.read()
                        if not ret:
                            print(f"[WARN] Frame not ready on attempt {i+1}")
                        time.sleep(0.1)
                    print("[INFO] Camera ready. Starting main loop.")

                    seen_colors = sweep_right_and_summarize(
                        cap,
                        steps=55,
                        step_dist=2,
                        speed=40,
                        pause=0.25,
                        show_preview=True
                    )
                    print(seen_colors)

                    cap.release()
                    cv2.destroyAllWindows()

                    if 'white' in seen_colors:
                        move(directions['left'], 200, 40, 0)
                        while True:
                            move(directions['left'], 1, 40, 0)

                            sensor_data = read_sensor_i2c()

                            print(sensor_data)

                            IR_RIGHT = sensor_data[0]
                            print("IR_LEFT:", IR_RIGHT)

                            if IR_RIGHT == '0':
                                confirm_count += 1
                            else:
                                confirm_count = 0
                            if confirm_count >= required_confirmations:
                                break
                        move(directions['left'], 50, 40, 0)
                        break
                    else:
                        while True:
                            move(directions['right'], 1, 40, 0)

                            sensor_data = read_sensor_i2c()

                            print(sensor_data)

                            IR_RIGHT = sensor_data[0]
                            print("IR_RIGHT:", IR_RIGHT)

                            if IR_RIGHT == '0':
                                confirm_count += 1
                            else:
                                confirm_count = 0

                            if confirm_count >= required_confirmations:
                                break


        move(directions['left'], 70, 40, 0)

        stop(0.2)

        while True:
            move(directions['forward'], 1, 20, 0)

            sensor_data = read_sensor_i2c()

            if sensor_data[6] == '0':
                break

        stop(0.2)

        move(directions['backward'], 10, 20, 0)

        stop(0.2)

        catch_ball()

        if ball_round == 0:
            move(directions['forward'], 250, 100, 0)
        elif ball_round == 1:
            move(directions['forward'], 200, 100, 0)
        else:
            pass

        stop(0.2)

        calibrate()

        stop(0.2)

        move(directions['backward'], 20, 30, 0)

        stop(0.2)

        calibrate()

        stop(0.2)

        print("[INFO] Warming up camera...")
        for i in range(10):
            ret, frame = cap.read()
            if not ret:
                print(f"[WARN] Frame not ready on attempt {i+1}")
            time.sleep(0.1)
        print("[INFO] Camera ready. Starting main loop.")

        move(directions['left'], 80, 30, 0)

        scan_step_count = 0
        max_scan_steps = 50
        scan_pause_duration = 0.4

        white_pin_lost_counter = 0
        white_pin_lost_threshold = 3

        print("Checking for red pin before scanning white pin...")

        if ball_round == 2:
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("cannot open camera")
                sys.exit(1)
                
        ret, frame = cap.read()
        if not ret:
            print("Can't read camera")
            cap.release()
            cv2.destroyAllWindows()
            exit()
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in red_contours:
            if cv2.contourArea(cnt) > 300:
                red_pin_found_first_round = True
                print("Red pin detected before scanning.")
                break
        else:
            print("No red pin detected before scanning.")

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Can't read camera")
                break

            height, width = frame.shape[:2]

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_white = np.array([0, 0, 180])
            upper_white = np.array([180, 60, 255])
            mask = cv2.inRange(hsv, lower_white, upper_white)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            centers = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 300:
                    M = cv2.moments(cnt)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        centers.append((cx, cy))
                        cv2.circle(frame, (cx, cy), 5, (255, 255, 0), -1)
            
            found_white = bool(centers)

            
            if first_round:
                if red_pin_found_first_round:
                    if found_white and not white_pin_detected_once:
                        print("[INFO] First white pin detected. Waiting for it to disappear...")
                        white_pin_detected_once = True
                        white_pin_lost_counter = 0

                    elif not found_white and white_pin_detected_once:
                        white_pin_lost_counter += 1
                        print(f"[DEBUG] White pin lost count: {white_pin_lost_counter}")

                        if white_pin_lost_counter >= white_pin_lost_threshold:
                            print("[ACTION] White pin disappeared. Now shooting!")

                            move(directions['left'], 70, 25, 0)
                            rotate("left", 2.8)
                            stop()
                            release_ball()
                            stop(1)
                            rotate("right", 2.8)
                            break

                    elif found_white and white_pin_detected_once:
                        white_pin_lost_counter = 0

                else:
                    if found_white:
                        print("[ACTION] No red pin. Shoot immediately!")

                        move(directions['right'], 100, 25, 0)

                        if found_left_black == False:
                            rotate("right", 1.4)
                            stop()
                            release_ball()
                            stop(1)
                            rotate("left", 1.4)
                        break

            else: 
                if red_pin_found_first_round:
                    if found_white:
                        print("[ACTION] Second round (red pin previously seen) and white pin detected → Shoot now!")
                        move(directions['left'], 250, 25, 0)
                        rotate("left", 1.4)
                        stop()
                        release_ball()
                        stop(1)
                        rotate("right", 1.4)
                        break
                    else:
                        print("[INFO] Second round red pin seen before but no white pin detected yet, keep scanning...")

                else:
                    if found_white :
                        print("[ACTION] No red pin round 2. Shoot immediately!")
                        move(directions['right'], 100, 25, 0)
                        rotate("right", 1.3)
                        stop()
                        release_ball()
                        stop(1)
                        rotate("left", 1.3)
                        break


            if scan_step_count >= max_scan_steps:
                print("No pin found within scan range, Stop!")
                stop()
                break

            print(f"[Step {scan_step_count}] → Left to White")
            move(directions['left'], 1, 25, 0)
            time.sleep(scan_pause_duration) 
            scan_step_count += 1

            cv2.putText(frame, f"White Pins: {len(centers)}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow("White Pin Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop()
                break

        cap.release()
        cv2.destroyAllWindows()
        release_ball()

        move(directions['backward'], 75, 50, 0) 

        calibrate()

        stop(1)

        if found_left_black == False:
            move(directions['left'], 50, 50, 0)

            while True:
                move(directions['left'], 1, 30, 0)

                sensor_data = read_sensor_i2c()

                if sensor_data[5] == '0':
                    break

        stop(0.2)

        move(directions['right'], 40, 50, 0)

        stop(0.2)

        move(directions['backward'], 200, 50, 0)

        while True:
            sensor_data = read_sensor_i2c()
            IR_1 = sensor_data[2]
            IR_4 = sensor_data[4]

            print(f"IR_1: {IR_1}, IR_4: {IR_4}")

            move(directions['backward'], 1, 100, 0)

            if IR_4 == '0' and IR_1 == '0':
                break

        move(directions['backward'], 15, 50, 0)

        stop(0.2)

        print(f'BALL {ball_round + 1} DONE')

        if found_left_black == True:
            release_ball()

    GPIO.cleanup()
    print('FRAME THREE PROGRAM DONE')