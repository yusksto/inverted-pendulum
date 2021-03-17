#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import smbus
import time
import threading
import numpy as np

PinA_1=6
PinB_1=13
PinC_1=19
PinD_1=26
PinA_2=10
PinB_2=22
PinC_2=9
PinD_2=11

moter_sleep_min = 0.001
moter_sleep_mid = 0.02
moter_sleep_max = 0.07

moter_steps = 200
operation_modes = 4
k = 2 * np.pi / moter_steps * operation_modes

def func(x):
	a = 0.4
	b = 2 / a
	return a * (1 - np.exp(-b * x)) / (1 + np.exp(-b * x))
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
	time.sleep(0.1)
def MOTER_LEFT():
	global v_theta_moter_left
	global theta_moter_left
	global motor_left_continue
	global powersave_left

	v_theta_moter_left = 0
	theta_moter_left = 0
	motor_left_continue = True
	powersave_left = False
	
	n = 0
	dt = 0
	k_1 = 2 * np.pi / moter_steps * operation_modes

	GPIO.output(PinD_1,GPIO.HIGH)
	while motor_left_continue == True:	
		theta_moter_left = k_1 * n
		if not v_theta_moter_left == 0:
			dt = k / v_theta_moter_left
		if powersave_left == True and moter_sleep_mid > np.abs(dt):
			powersave_left = False
			GPIO.output(PinC_1,GPIO.LOW)
		if powersave_left == False and moter_sleep_mid < np.abs(dt):
			powersave_left = True
			GPIO.output(PinC_1,GPIO.HIGH)
		if np.abs(dt) < moter_sleep_min or moter_sleep_max < np.abs(dt):
			time.sleep(moter_sleep_min)
			continue
		i = n % 4
		if dt > 0:
			if i == 0:
				GPIO.output(PinA_1,GPIO.HIGH)
				time.sleep(np.abs(dt))
				n += 1
			elif i == 1:
				GPIO.output(PinB_1,GPIO.HIGH)
				time.sleep(np.abs(dt))
				n += 1
			elif i == 2:
				GPIO.output(PinA_1,GPIO.LOW)
				time.sleep(np.abs(dt))
				n += 1
			elif i == 3:
				GPIO.output(PinB_1,GPIO.LOW)
				time.sleep(np.abs(dt))
				n += 1
		elif dt < 0:
			if i == 0:
				GPIO.output(PinB_1,GPIO.HIGH)
				time.sleep(np.abs(dt))
				n -= 1
			elif i == 1:
				GPIO.output(PinA_1,GPIO.LOW)
				time.sleep(np.abs(dt))
				n -= 1
			elif i == 2:
				GPIO.output(PinB_1,GPIO.LOW)
				time.sleep(np.abs(dt))
				n -= 1
			elif i == 3:
				GPIO.output(PinA_1,GPIO.HIGH)
				time.sleep(np.abs(dt))
				n -= 1
	GPIO.output(PinD_1,GPIO.LOW)
def MOTER_RIGHT():
	global v_theta_moter_right
	global theta_moter_right
	global motor_right_continue
	global powersave_right

	v_theta_moter_right = 0
	theta_moter_right = 0
	motor_right_continue = True
	powersave_right = False

	GPIO.output(PinD_2,GPIO.HIGH)

	n = 0
	dt = 0
	k_1 = 2 * np.pi / moter_steps * operation_modes

	while motor_right_continue == True:
		theta_moter_right = k_1 * n
		if not v_theta_moter_right == 0:
			dt = k / v_theta_moter_right
		if powersave_right == True and moter_sleep_mid > np.abs(dt):
			powersave_right = False
			GPIO.output(PinC_2,GPIO.LOW)
		if powersave_right == False and moter_sleep_mid < np.abs(dt):
			powersave_right = True
			GPIO.output(PinC_2,GPIO.HIGH)
		if np.abs(dt) < moter_sleep_min or moter_sleep_max < np.abs(dt):
			time.sleep(moter_sleep_min)
			continue
		i = n % 4
		if dt > 0:
			if i == 0:
				GPIO.output(PinA_2,GPIO.HIGH)
				time.sleep(np.abs(dt))
				n += 1
			elif i == 1:
				GPIO.output(PinB_2,GPIO.HIGH)
				time.sleep(np.abs(dt))
				n += 1
			elif i == 2:
				GPIO.output(PinA_2,GPIO.LOW)
				time.sleep(np.abs(dt))
				n += 1
			elif i == 3:
				GPIO.output(PinB_2,GPIO.LOW)
				time.sleep(np.abs(dt))
				n += 1
		elif dt < 0:
			if i == 0:
				GPIO.output(PinB_2,GPIO.HIGH)
				time.sleep(np.abs(dt))
				n -= 1
			elif i == 1:
				GPIO.output(PinA_2,GPIO.LOW)
				time.sleep(np.abs(dt))
				n -= 1
			elif i == 2:
				GPIO.output(PinB_2,GPIO.LOW)
				time.sleep(np.abs(dt))
				n -= 1
			elif i == 3:
				GPIO.output(PinA_2,GPIO.HIGH)
				time.sleep(np.abs(dt))
				n -= 1
	GPIO.output(PinD_2,GPIO.LOW)
def CLEAR_GPIO():
	GPIO.cleanup()
	time.sleep(0.1)
def SET_BMX055():
	# Get I2C bus
	global bus
	bus = smbus.SMBus(1)
	# BMX055 Accl address, 0x19(24)
	# Select PMU_Range register, 0x0F(15)
	#		0x03(03)	Range = +/- 2g
	bus.write_byte_data(0x19, 0x0F, 0x03)
	# BMX055 Accl address, 0x19(24)
	# Select PMU_BW register, 0x10(16)
	#		0x13(13)	Bandwidth = 250 Hz
	bus.write_byte_data(0x19, 0x10, 0x08)
	# BMX055 Accl address, 0x19(24)
	# Select PMU_LPW register, 0x11(17)
	#		0x00(00)	Normal mode, Sleep duration = 0.5ms
	bus.write_byte_data(0x19, 0x11, 0x00)
	# BMX055 Gyro address, 0x69(104)
	# Select Range register, 0x0F(15)
	#		0x02(02)	Full scale = +/- 500 degree/s
	bus.write_byte_data(0x69, 0x0F, 0x02)
	# BMX055 Gyro address, 0x69(104)
	# Select Bandwidth register, 0x10(16)
	#		0x04(04)	ODR = 200 Hz
	bus.write_byte_data(0x69, 0x10, 0x04)
	# BMX055 Gyro address, 0x69(104)
	# Select LPM1 register, 0x11(17)
	#		0x00(00)	Normal mode, Sleep duration = 2ms
	bus.write_byte_data(0x69, 0x11, 0x00)
	time.sleep(0.1)
def GET_XACCL():
	data = bus.read_i2c_block_data(0x19, 0x02, 2)
	xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16
	if xAccl > 2047 :
		xAccl -= 4096
	return xAccl * 0.0098
def GET_YACCL():
	data = bus.read_i2c_block_data(0x19, 0x04, 2)
	yAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16
	if yAccl > 2047 :
		yAccl -= 4096
	return yAccl * 0.0098
def GET_ZACCL():
	data = bus.read_i2c_block_data(0x19, 0x06, 2)
	zAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16
	if zAccl > 2047 :
		zAccl -= 4096
	return zAccl * 0.0098
def GET_XGYRO():
	data = bus.read_i2c_block_data(0x69, 0x02, 2)
	xGyro = data[1] * 256 + data[0]
	if xGyro > 32767 :
		xGyro -= 65536
	return xGyro * 0.0153/360*2*3.14159
def GET_YGYRO():
	data = bus.read_i2c_block_data(0x69, 0x04, 2)
	yGyro = data[1] * 256 + data[0]
	if yGyro > 32767 :
		yGyro -= 65536
	return yGyro * 0.0153/360*2*3.14159
def GET_ZGYRO():
	data = bus.read_i2c_block_data(0x69, 0x06, 2)
	zGyro = data[1] * 256 + data[0]
	if zGyro > 32767 :
		zGyro -= 65536
	return zGyro * 0.0153/360*2*3.14159
def KEYBOARD_OPERATION():
	global v_right
	global v_left
	global keyboard_operation_continue
	global x
	global theta_moter_left
	global theta_moter_right
	v_left = 0
	v_right = 0
	v_slide = 0.25
	keyboard_operation_continue = True

	a = 0
	b = 0

	while keyboard_operation_continue == True:
		keyboard_input = input()
		print(keyboard_input)
		if keyboard_input == "x":
			x = 0
			theta_moter_left = 0
			theta_moter_right = 0
		elif keyboard_input == "w" and (not a == 1):
			a += 1
		elif keyboard_input == "s" and (not a == -1):
			a -= 1
		elif keyboard_input == "d" and (not b == 1):
			b += 1
		elif keyboard_input == "a" and (not b == -1):
			b -= 1
		else:
			a = 0
			b = 0
		v_left = v_slide * (2 * a + b)
		v_right = v_slide * (2 * a - b)
def CONTROL():
	global theta
	theta = 0
	global v_theta
	v_theta = 0
	global x
	x = 0
	global v_x
	v_x = 0
	global a_x
	a_x = 0
	global phi
	phi = 0
	global v_phi
	v_phi = 0
	global control_continue
	control_continue = True
	global t
	t = 0
	global dt_sleep
	dt_sleep = 0
	global x_1
	x_1 = 0
	global x_2
	x_2 = 0
	global v_theta_moter_left
	global v_theta_moter_right

	theta = 0
	v_theta = 0



	theta_zero_detection = False
	cal = True

	theta_GYRO = 0
	theta_ACCL = 0

	n = 5
	p_1 = 0.05 #角度推定相補フィルター定数
	p_2 = 0.9 #ゼロ度検出用ローパスフィルタ定数
	p_3 = 0.5 #角速度ローパスフィルタ定数
	p_4 = 0.9 #加速度による角度推定用ローパスフィルタ定数
	p_5 = 0.7 #(角度推定相補フィルター定数"p_1")用ローパスフィルタ定数
	p_6 = 0.9 #位置推定相補フィルター用定数
	p_7 = 0.8 #加速度ローパスフィルタ


	#筐体情報
	m = 0.78 #筐体重さ
	R = 0.05 #筐体重心高さ
	I = 0.10617 #筐体回転軸回り慣性モーメント
	r = 0.045 #車輪半径
	l = 0.115 #車輪幅

	#制御パラメータ
	k_2 = 15 #角速度項
	k_1 = 9.80665 + m * R / 4 / I * k_2 * k_2 #角度項
	k_3 = 1 #座標項
	k_4 = 4 #速度項


	a_y = 0
	a_z = 0

	theta_zero_judge = 10

	dt = 0.007
	dt_sleep = dt
	t_start = time.time()
	while control_continue == True:
		for i in range(n):
			#センサーより値を取得、補正、フィルタリング
			a_y_raw = (-GET_XACCL() - 0.239738) * 0.980209
			a_z_raw = (GET_ZACCL() + 0.285688) * 0.984507			
			v_theta_raw = -GET_YGYRO() + 0.0000822816
			
			a_y = a_y * p_7 + a_y_raw * (1 - p_7)
			a_z = a_z * p_7 + a_z_raw * (1 - p_7)
			v_theta = v_theta * p_3 + v_theta_raw * (1 - p_3)

			#加速度から角度を算出
			theta_ACCL = theta_ACCL * p_4 + (-np.arctan(a_y/a_z) - 0.007725428306555) * (1 - p_4)
			#角速度を積分し角度を算出（補正付き）
			theta_GYRO += v_theta * dt
			p_1 = p_1 * p_5 + (0.05 * np.exp(-11.09*np.power(9.80665 - np.sqrt(a_y*a_y+a_z*a_z), 2))) * (1 - p_5)
			theta = (theta + v_theta * dt) * (1 - p_1) + theta_ACCL * p_1
			#ゼロ度検出
			if theta_zero_detection == False:
				theta_zero_judge = theta_zero_judge * p_2  + np.abs(theta_ACCL) * (1 - p_2)
				if theta_zero_judge < 0.005:
					theta = 0
					theta_GYRO = 0
					theta_zero_detection = True

			if theta_zero_detection == True:
				a_x = k_1 * theta + k_2 * v_theta + k_3 * x + k_4 * v_x
				x_2 = theta_moter_left * r
				x_1 = theta_moter_right * r
				x = (x + v_x * dt) * p_6 + (x_1 + x_2) / 2 * (1 - p_6)
				v_x += a_x * dt

				v_theta_moter_left = (v_x + v_left) / r
				v_theta_moter_right = (v_x + v_right) / r
				x -= (v_left + v_right) / 2 * dt

				phi = (x_2 - x_1) / l

			t += dt
			time.sleep(dt_sleep)
		#周波数調整
		dt_sleep = dt - (time.time() - t_start - t) / n
		if dt_sleep < 0.001:
			dt_sleep = 0.001
		if dt < dt_sleep:
			dt_sleep = dt
if __name__ == '__main__':
	try:
		f = open('data.dat', 'w')
		#GPIO設定
		SET_GPIO()
		#i2cとbmx055設定
		SET_BMX055()
		#各スレッドでスタート
		moter_left = threading.Thread(target=MOTER_LEFT)
		moter_left.start()
		moter_right = threading.Thread(target=MOTER_RIGHT)
		moter_right.start()
		keyboard_operation = threading.Thread(target=KEYBOARD_OPERATION)
		keyboard_operation.start()
		time.sleep(0.1)
		control = threading.Thread(target=CONTROL)
		control.start()
		while True:
			f.write(str(t) + "	" + str(dt_sleep) + "	" + str(theta) + "	" + str(v_theta) + "	" + str(phi) + "	" + str(v_phi) + "	" + str(x) + "	" + str(x_1) + "	" + str(x_2) + "	" + str(v_x) + "	" + str(a_x) + '\n')
			time.sleep(0.01)
	except KeyboardInterrupt:
		print("\nCtl+C")
	except Exception as e:
		print(str(e))
	finally:
		#スレッド終了
		motor_left_continue = False
		motor_right_continue = False
		control_continue = False
		keyboard_operation_continue = False
		moter_left.join()
		moter_right.join()
		control.join()
		keyboard_operation.join()
		#gpio終了
		CLEAR_GPIO()
		#i2c終了
		bus.close()
		f.close()
		print("\nexit program")