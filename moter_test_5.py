#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import smbus
import time
from time import sleep
import threading
import numpy as np

motor_drive_left = True
motor_drive_right = True
dt_moter_left = 0
dt_moter_right = 0

PinA_1=6
PinB_1=13
PinC_1=19
PinD_1=26
PinA_2=10
PinB_2=22
PinC_2=9
PinD_2=11

def SET_GPIO():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(PinA_1, GPIO.OUT)
	GPIO.setup(PinB_1, GPIO.OUT)
	GPIO.setup(PinC_1, GPIO.OUT)
	GPIO.setup(PinD_1, GPIO.OUT)
	GPIO.setup(PinA_2, GPIO.OUT)
	GPIO.setup(PinB_2, GPIO.OUT)
	GPIO.setup(PinC_2, GPIO.OUT)
	GPIO.setup(PinD_2, GPIO.OUT)
def MOTER_LEFT():
	global dt_moter_left
	dt_moter_left = 0
	powersave_left = False
	GPIO.output(PinD_1,GPIO.HIGH)
	while motor_drive_left == True:
		if not (0.002 < np.abs(dt_moter_left) and np.abs(dt_moter_left) < 0.1):
			if powersave_left == False:
				powersave_left = True
				GPIO.output(PinC_1,GPIO.HIGH)
			time.sleep(0.002)
			continue
		if powersave_left == True:
			powersave_left = False
			GPIO.output(PinC_1,GPIO.LOW)
		if 0.002 < dt_moter_left and dt_moter_left < 0.1:
			GPIO.output(PinA_1,GPIO.HIGH)
			sleep(np.abs(dt_moter_left))
		if 0.002 < dt_moter_left and dt_moter_left < 0.1:
			GPIO.output(PinB_1,GPIO.HIGH)
			sleep(np.abs(dt_moter_left))
		if 0.002 < dt_moter_left and dt_moter_left < 0.1:
			GPIO.output(PinA_1,GPIO.LOW)
			sleep(np.abs(dt_moter_left))
		if 0.002 < dt_moter_left and dt_moter_left < 0.1:
			GPIO.output(PinB_1,GPIO.LOW)
			sleep(np.abs(dt_moter_left))
		if -0.1 < dt_moter_left and dt_moter_left < -0.002:
			GPIO.output(PinB_1,GPIO.HIGH)
			sleep(np.abs(dt_moter_left))
		if -0.1 < dt_moter_left and dt_moter_left < -0.002:
			GPIO.output(PinA_1,GPIO.HIGH)
			sleep(np.abs(dt_moter_left))
		if -0.1 < dt_moter_left and dt_moter_left < -0.002:
			GPIO.output(PinB_1,GPIO.LOW)
			sleep(np.abs(dt_moter_left))
		if -0.1 < dt_moter_left and dt_moter_left < -0.002:
			GPIO.output(PinA_1,GPIO.LOW)
			sleep(np.abs(dt_moter_left))
	GPIO.output(PinD_1,GPIO.LOW)
def MOTER_RIGHT():
	global dt_moter_right
	dt_moter_left = 0
	powersave_right = False
	GPIO.output(PinD_2,GPIO.HIGH)
	while motor_drive_right == True:
		if not (0.002 < np.abs(dt_moter_right) and np.abs(dt_moter_right) < 0.1):
			if powersave_right == False:
				powersave_right = True
				GPIO.output(PinC_2,GPIO.HIGH)
			time.sleep(0.002)
			continue
		if powersave_right == True:
			powersave_right = False
			GPIO.output(PinC_2,GPIO.LOW)
		if 0.002 < dt_moter_right and dt_moter_right < 0.1:
			GPIO.output(PinA_2,GPIO.HIGH)
			sleep(np.abs(dt_moter_right))
		if 0.002 < dt_moter_right and dt_moter_right < 0.1:
			GPIO.output(PinB_2,GPIO.HIGH)
			sleep(np.abs(dt_moter_right))
		if 0.002 < dt_moter_right and dt_moter_right < 0.1:
			GPIO.output(PinA_2,GPIO.LOW)
			sleep(np.abs(dt_moter_right))
		if 0.002 < dt_moter_right and dt_moter_right < 0.1:
			GPIO.output(PinB_2,GPIO.LOW)
			sleep(np.abs(dt_moter_right))
		if -0.1 < dt_moter_right and dt_moter_right < -0.002:
			GPIO.output(PinB_2,GPIO.HIGH)
			sleep(np.abs(dt_moter_right))
		if -0.1 < dt_moter_right and dt_moter_right < -0.002:
			GPIO.output(PinA_2,GPIO.HIGH)
			sleep(np.abs(dt_moter_right))
		if -0.1 < dt_moter_right and dt_moter_right < -0.002:
			GPIO.output(PinB_2,GPIO.LOW)
			sleep(np.abs(dt_moter_right))
		if -0.1 < dt_moter_right and dt_moter_right < -0.002:
			GPIO.output(PinA_2,GPIO.LOW)
			sleep(np.abs(dt_moter_right))
	GPIO.output(PinD_2,GPIO.LOW)
def CLEAR_GPIO():
	GPIO.cleanup()
	sleep(0.1)
if __name__ == '__main__':
	SET_GPIO()
	dt = 0.01
	v_theta_1 = 7
	v_theta_2 = -7
	a_theta_1 = -0.5
	a_theta_2 = 0.5
	dt_moter_left = np.pi / 25 / v_theta_1
	dt_moter_right = np.pi / 25 / v_theta_2
	t1 = threading.Thread(target=MOTER_LEFT)
	t2 = threading.Thread(target=MOTER_RIGHT)
	t1.start()
	t2.start()
	try:
		while True:
			dt_moter_left = np.pi / 25 / v_theta_1
			dt_moter_right = np.pi / 25 / v_theta_2
			v_theta_1 += a_theta_1 * dt
			v_theta_2 += a_theta_2 * dt
			sleep(dt)
	except KeyboardInterrupt:
		print("\nCtl+C")
	except Exception as e:
		print(str(e))
	finally:
		motor_drive_left = False
		motor_drive_right = False
		t1.join()
		t2.join()
		GPIO.cleanup()
		sleep(0.1)
		print("\nexit program")
