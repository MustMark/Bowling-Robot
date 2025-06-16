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


if __name__ == '__main__':
    rospy.init_node('bid_team_node', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)

    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(1)

    GPIO.output(24, GPIO.LOW)
    print("Press Start Button !")
    
    while True:
        sensor_data = read_sensor_i2c()

        if sensor_data[7] == '0':
            GPIO.output(24, GPIO.HIGH)
            break

    for ball_round in range(2):

        required_confirmations = 3
        confirm_count = 0

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

        if ball_round == 0:
            move(directions['forward'], 100, 60, 0)
        elif ball_round == 1:
            move(directions['forward'], 150, 60, 0)
        else:
            pass

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

        stop(2)

        if ball_round == 0:
            move(directions['left'], 100, 60, 0)
        elif ball_round == 1:
            move(directions['left'], 400, 60, 0)
        else:
            pass

        release_ball()

        stop(1)

        while True:
            move(directions['left'], 1, 50, 0)

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

        print(f'BALL {ball_round + 1} DONE')
    
    GPIO.cleanup()
    print("PROGRAM DONE")