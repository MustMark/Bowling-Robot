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
        set_velocity.publish(0, 0, 0.1)
    else:
        set_velocity.publish(0, 0, -0.1)
    rospy.sleep(duration)

def calibrate():
    while True:
        sensor_data = read_sensor_i2c()
        IR_1 = sensor_data[1]
        IR_4 = sensor_data[4]

        print(f"IR_1: {IR_1}, IR_4: {IR_4}")

        if IR_4 == '1' and IR_1 == '1':
            move(directions['forward'], 1, 10, 0)

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


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("cannot open camera")
        exit()
    required_confirmations = 3
    confirm_count = 0
    tolerance = 10
    min_pin_count = 1
    max_seen_pins = 0
    mode = "scan"
    rospy.init_node('bid_team_node', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)

    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(1)

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

    stop(0.2)

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

    move(directions['forward'], 150, 100, 0)

    stop(0.2)

    calibrate()

    stop(0.2)

    move(directions['backward'], 20, 30, 0)

    stop(0.2)

    calibrate()

    stop(0.2)

    mode = "scan"
    scan_pause_duration = 0.6
    scan_step_count = 0
    max_scan_steps = 40
    green_confirm_count = 0
    green_required_confirmations = 3
    confirm_count_pin = 0
    required_confirm_pin = 3

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't read camera")
            break

        height, width = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        target_center = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300 and area > max_area:
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    target_center = (cx, cy)
                    max_area = area

        if mode == "scan":
            if target_center:
                green_confirm_count += 1
                print("Found Green switch to dx")
                if green_confirm_count >= green_required_confirmations:
                    print("Found Green switch to dx")
                    stop()
                    mode = "rotate"
                    continue
            else:
                green_confirm_count = 0

            if scan_step_count >= max_scan_steps:
                print("Scan NotFound stop")
                stop()
                break

            print(f"[Scan {scan_step_count}] → Left to Green")
            move(directions['left'], 1, 20, 0)
            time.sleep(scan_pause_duration)
            scan_step_count += 1

        elif mode == "rotate":
            if target_center:
                avg_x, avg_y = target_center
                dx = avg_x - width // 2

                cv2.circle(frame, target_center, 10, (0, 255, 0), -1)
                cv2.line(frame, (width // 2, height // 2), target_center, (0, 255, 255), 1)
                cv2.putText(frame, f"dx = {dx}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                if abs(dx) > tolerance:
                    if  -3 <= dx <= 3:
                        confirm_count_pin += 1
                        print(f"Confirm Count: {confirm_count_pin}")
                        if confirm_count_pin >= required_confirm_pin:
                            stop()
                            print("100% Release Ball")
                            release_ball()
                            break
                    else:
                        confirm_count_pin = 0 

                        if -20 <= dx < 10:
                            print("dx < 0 (a little) → move left slowly")
                            move(directions['left'], 1, 5, 0)

                        elif dx < -30:
                            print("dx < 0 (a lot) → move left quickly")
                            move(directions['left'], 2, 10, 0)

                        elif 10 < dx <= 20:
                            print("dx > 0 (a little) → move right slowly")
                            move(directions['right'], 1, 5, 0)

                        elif dx > 30:
                            print("dx > 0 (a lot) → move right quickly")
                            move(directions['right'], 2, 10, 0)

                        stop(0.3)
                else:
                    stop()
                    print("Found Green!!!")
                    release_ball()
                    break
            else:
                print("Scan again !")
                mode = "scan"
                scan_step_count = 0

        cv2.imshow("Green Pin Align", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop()
            break

    cap.release()
    cv2.destroyAllWindows()

    release_ball()

    stop(1)

    move(directions['left'], 50, 90, 0)

    while True:
        move(directions['left'], 1, 30, 0)

        sensor_data = read_sensor_i2c()

        if sensor_data[5] == '0':
            break

    stop(0.2)

    move(directions['right'], 30, 50, 0)

    stop(0.2)

    move(directions['backward'], 200, 50, 0)

    while True:
        sensor_data = read_sensor_i2c()
        IR_1 = sensor_data[2]
        IR_4 = sensor_data[4]

        print(f"IR_1: {IR_1}, IR_4: {IR_4}")

        move(directions['backward'], 1, 30, 0)

        if IR_4 == '0' and IR_1 == '0':
            break

    move(directions['backward'], 40, 50, 0)

    print('DONE')