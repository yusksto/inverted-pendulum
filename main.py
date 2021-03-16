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
moter_sleep_max = 0.1

moter_steps = 200
operation_modes = 4
k = 2 * np.pi / moter_steps * operation_modes

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
def MOTER_SLEEP_LEFT(dt):
	if dt < moter_sleep_max:
		time.sleep(dt)
	else:
		powersave_left = True
		GPIO.output(PinC_1,GPIO.HIGH)
		n = int(dt / moter_sleep_min)
		for i in range(n):
			time.sleep(moter_sleep_min)
			if k / v_theta_moter_left < dt:
				powersave_left = False
				GPIO.output(PinC_1,GPIO.LOW)
				break
		powersave_left = False
		GPIO.output(PinC_1,GPIO.LOW)
def MOTER_SLEEP_RIGHT(dt):
	if dt < moter_sleep_max:
		time.sleep(dt)
	else:
		powersave_right = True
		GPIO.output(PinC_2,GPIO.HIGH)
		n = int(dt / moter_sleep_min)
		for i in range(n):
			time.sleep(moter_sleep_min)
			if k / v_theta_moter_right < dt:
				powersave_right = False
				GPIO.output(PinC_2,GPIO.LOW)
				break
		powersave_right = False
		GPIO.output(PinC_2,GPIO.LOW)
def MOTER_LEFT():
	global v_theta_moter_left
	global theta_moter_left
	global motor_drive_left
	global powersave_left

	v_theta_moter_left = 0
	theta_moter_left = 0
	motor_drive_left = True
	powersave_left = False
	GPIO.output(PinD_1,GPIO.HIGH)

	n = 0
	dt = 0
	k_1 = 2 * np.pi / 200

	while motor_drive_left == True:	
		theta_moter_left = k_1 * n
		if not v_theta_moter_left == 0:
			dt = k / v_theta_moter_left		
		if np.abs(dt) < moter_sleep_min:
			time.sleep(moter_sleep_min)
			continue
		i = n % 4		
		if i == 0 and dt > 0:
			GPIO.output(PinA_1,GPIO.HIGH)
			MOTER_SLEEP_LEFT(np.abs(dt))
			n += 1
		elif i == 1 and dt > 0:
			GPIO.output(PinB_1,GPIO.HIGH)
			MOTER_SLEEP_LEFT(np.abs(dt))
			n += 1
		elif i == 2 and dt > 0:
			GPIO.output(PinA_1,GPIO.LOW)
			MOTER_SLEEP_LEFT(np.abs(dt))
			n += 1
		elif i == 3 and dt > 0:
			GPIO.output(PinB_1,GPIO.LOW)
			MOTER_SLEEP_LEFT(np.abs(dt))
			n += 1
		elif i == 0 and dt < 0:
			GPIO.output(PinB_1,GPIO.HIGH)
			MOTER_SLEEP_LEFT(np.abs(dt))
			n -= 1
		elif i == 1 and dt < 0:
			GPIO.output(PinA_1,GPIO.LOW)
			MOTER_SLEEP_LEFT(np.abs(dt))
			n -= 1
		elif i == 2 and dt < 0:
			GPIO.output(PinB_1,GPIO.LOW)
			MOTER_SLEEP_LEFT(np.abs(dt))
			n -= 1
		elif i == 3 and dt < 0:
			GPIO.output(PinA_1,GPIO.HIGH)
			MOTER_SLEEP_LEFT(np.abs(dt))
			n -= 1
	GPIO.output(PinD_1,GPIO.LOW)
def MOTER_RIGHT():
	global v_theta_moter_right
	global theta_moter_right
	global motor_drive_right
	global powersave_right

	v_theta_moter_right = 0
	theta_moter_right = 0
	motor_drive_right = True
	powersave_right = False

	GPIO.output(PinD_2,GPIO.HIGH)

	n = 0
	dt = 0
	k_1 = 2 * np.pi / 200

	while motor_drive_right == True:
		theta_moter_right = k_1 * n
		if not v_theta_moter_right == 0:
			dt = k / v_theta_moter_right
		if np.abs(dt) < moter_sleep_min:
			time.sleep(moter_sleep_min)
			continue
		i = n % 4
		if i == 0 and dt > 0:
			GPIO.output(PinA_2,GPIO.HIGH)
			MOTER_SLEEP_RIGHT(np.abs(dt))
			n += 1
		elif i == 1 and dt > 0:
			GPIO.output(PinB_2,GPIO.HIGH)
			MOTER_SLEEP_RIGHT(np.abs(dt))
			n += 1
		elif i == 2 and dt > 0:
			GPIO.output(PinA_2,GPIO.LOW)
			MOTER_SLEEP_RIGHT(np.abs(dt))
			n += 1
		elif i == 3 and dt > 0:
			GPIO.output(PinB_2,GPIO.LOW)
			MOTER_SLEEP_RIGHT(np.abs(dt))
			n += 1
		elif i == 0 and dt < 0:
			GPIO.output(PinB_2,GPIO.HIGH)
			MOTER_SLEEP_RIGHT(np.abs(dt))
			n -= 1
		elif i == 1 and dt < 0:
			GPIO.output(PinA_2,GPIO.LOW)
			MOTER_SLEEP_RIGHT(np.abs(dt))
			n -= 1
		elif i == 2 and dt < 0:
			GPIO.output(PinB_2,GPIO.LOW)
			MOTER_SLEEP_RIGHT(np.abs(dt))
			n -= 1
		elif i == 3 and dt < 0:
			GPIO.output(PinA_2,GPIO.HIGH)
			MOTER_SLEEP_RIGHT(np.abs(dt))
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
def CAL_THETA():
	global theta
	global v_theta
	global theta_zero_detection
	global cal

	theta = 0
	v_theta = 0
	theta_zero_detection = False
	cal = True

	theta_GYRO = 0
	theta_ACCL = 0

	n = 2
	alpha = 0.05 #角度推定相補フィルター定数
	beta = 0.9 #ゼロ度検出用ローパスフィルタ定数
	gamma = 0.5 #角速度ローパスフィルタ定数
	delta = 0.9 #加速度による角度推定用ローパスフィルタ定数
	zeta = 0.9 #(角度推定相補フィルター定数"alpha")用ローパスフィルタ定数

	theta_zero_judge = 0

	dt = 0.007
	dt_sleep = dt
	t = 0
	t_start = time.time()
	while cal == True:
		for i in range(n):
			#加速度取得と補正
			a_x = (-GET_XACCL() - 0.239738) * 0.980209
			a_z = (GET_ZACCL() + 0.285688) * 0.984507
			#加速度から角度を算出
			theta_ACCL = theta_ACCL * delta + (-np.arctan(a_x/a_z) - 0.010725428306555) * (1 - delta)
			#角速度を積分し角度を算出（補正付き）
			v_theta = v_theta * gamma + (-GET_YGYRO() + 0.0000822816) * (1 - gamma)
			theta_GYRO += v_theta * dt
			alpha = alpha * zeta + (0.05 * np.exp(-11.09*np.power(9.80665 - np.sqrt(a_x*a_x+a_z*a_z), 2))) * (1 - zeta)
			theta = (theta + v_theta * dt) * (1 - alpha) + theta_ACCL * alpha
			#ゼロ度検出
			theta_zero_judge = theta_zero_judge * beta  + np.abs(theta_ACCL) * (1 - beta)
			if theta_zero_judge < 0.005:
				theta = 0
				theta_GYRO = 0
				theta_zero_detection = True

			t += dt
			time.sleep(dt_sleep)
		#周波数調整
		dt_sleep = dt - (time.time() - t_start - t) / n
		if dt_sleep < 0.001:
			dt_sleep = 0.001
		if dt < dt_sleep:
			dt_sleep = dt
def KEYBOARD_CONTROL():
	global v_right
	global v_left
	global keyboard
	v_left = 0
	v_right = 0
	v_slide = 0.1
	keyboard = True

	a = 0
	b = 0

	while keyboard == True:
		keyboard_input = input("keyboard control w,a,s,d")
		print(keyboard_input)		
		if keyboard_input == "w" and (not a == 1):
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
if __name__ == '__main__':
	try:
		f = open('data.dat', 'w')
		#GPIO設定
		SET_GPIO()
		#i2cとbmx055設定
		SET_BMX055()
		#モーター駆動と角度計算を別スレッドでスタート
		cal_theta = threading.Thread(target=CAL_THETA)
		cal_theta.start()
		keyboard_control = threading.Thread(target=KEYBOARD_CONTROL)
		keyboard_control.start()
		while True:#リセット用ループ
			SET_GPIO()			
			moter_left = threading.Thread(target=MOTER_LEFT)
			moter_right = threading.Thread(target=MOTER_RIGHT)
			motor_drive_left = True
			motor_drive_right = True
			moter_left.start()
			moter_right.start()
			#角度ゼロの検出待ち
			print("stand by")
			theta_zero_detection = False
			while theta_zero_detection == False:
				time.sleep(0.1)
			print("start control program")

			m = 0.78
			R = 0.05
			I = 0.10617
			r = 0.045

			k_2 = 15 #角速度項
			k_1 = 9.80665 + m * R / 4 / I * k_2 * k_2 #角度項
			k_3 = 3 #座標項
			k_4 = 5 #速度項

			a_x = 0
			v_x = 0.001
			x = 0

			dt = 0.007
			dt_sleep = dt
			t = 0
			t_start = time.time()
			n = 2
			while True:
				for i in range(n):
					theta_zero_detection == False
					a_x = k_1 * theta + k_2 * v_theta + k_4 * v_x					
					x += v_x * dt
					v_x += a_x * dt
					x_1 = theta_moter_left * r
					x_2 = theta_moter_right * r

					v_theta_moter_left = (v_x - v_left) / r
					v_theta_moter_right = (v_x - v_right) / r

					t += dt
					time.sleep(dt_sleep)
					if np.pi / 25 / v_x * r > moter_sleep_max * 2 and theta_zero_detection == True:
						v_x = 0
					#ログ出力
					f.write(str(t) + "	" + str(dt_sleep) + "	" + str(theta) + "	" + str(v_theta) + "	" + str(x) + "	" + str(x_1) + "	" + str(x_2) + "	" + str(v_x) + "	" + str(a_x) + '\n')
				#周波数調整			
				dt_sleep = dt - (time.time() - t_start - t) / n
				if dt_sleep < 0.001:
					dt_sleep = 0.001
				if dt < dt_sleep:
					dt_sleep = dt
				
				#リセット判定
				if np.abs(theta) > np.pi / 3:
					print("reset")
					motor_drive_left = False
					motor_drive_right = False
					moter_left.join()
					moter_right.join()
					theta_zero_detection = False
					CLEAR_GPIO()
					break
	except KeyboardInterrupt:
		print("\nCtl+C")
	except Exception as e:
		print(str(e))
	finally:
		#スレッド終了
		motor_drive_left = False
		motor_drive_right = False
		cal = False
		keyboard = False
		moter_left.join()
		moter_right.join()
		cal_theta.join()
		keyboard_control.join()
		#gpio終了
		CLEAR_GPIO()
		#i2c終了
		bus.close()
		f.close()
		print("\nexit program")