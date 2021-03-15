#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import smbus
import time
from time import sleep
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

moter_sleep_min = 0.002
moter_sleep_max = 0.1

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
	global motor_drive_left
	global powersave_left
	dt_moter_left = 0
	motor_drive_left = True
	powersave_left = False
	GPIO.output(PinD_1,GPIO.HIGH)
	while motor_drive_left == True:
		if not (moter_sleep_min < np.abs(dt_moter_left) and np.abs(dt_moter_left) < moter_sleep_max):
			if powersave_left == False:
				powersave_left = True
				GPIO.output(PinC_1,GPIO.HIGH)
			time.sleep(moter_sleep_min)
			continue
		if powersave_left == True:
			powersave_left = False
			GPIO.output(PinC_1,GPIO.LOW)
		if moter_sleep_min < dt_moter_left and dt_moter_left < moter_sleep_max:
			GPIO.output(PinA_1,GPIO.HIGH)
			sleep(np.abs(dt_moter_left))
		if moter_sleep_min < dt_moter_left and dt_moter_left < moter_sleep_max:
			GPIO.output(PinB_1,GPIO.HIGH)
			sleep(np.abs(dt_moter_left))
		if moter_sleep_min < dt_moter_left and dt_moter_left < moter_sleep_max:
			GPIO.output(PinA_1,GPIO.LOW)
			sleep(np.abs(dt_moter_left))
		if moter_sleep_min < dt_moter_left and dt_moter_left < moter_sleep_max:
			GPIO.output(PinB_1,GPIO.LOW)
			sleep(np.abs(dt_moter_left))
		if -moter_sleep_max < dt_moter_left and dt_moter_left < -moter_sleep_min:
			GPIO.output(PinB_1,GPIO.HIGH)
			sleep(np.abs(dt_moter_left))
		if -moter_sleep_max < dt_moter_left and dt_moter_left < -moter_sleep_min:
			GPIO.output(PinA_1,GPIO.HIGH)
			sleep(np.abs(dt_moter_left))
		if -moter_sleep_max < dt_moter_left and dt_moter_left < -moter_sleep_min:
			GPIO.output(PinB_1,GPIO.LOW)
			sleep(np.abs(dt_moter_left))
		if -moter_sleep_max < dt_moter_left and dt_moter_left < -moter_sleep_min:
			GPIO.output(PinA_1,GPIO.LOW)
			sleep(np.abs(dt_moter_left))
	GPIO.output(PinD_1,GPIO.LOW)
def MOTER_RIGHT():
	global dt_moter_right
	global motor_drive_right
	global powersave_right
	dt_moter_right = 0
	motor_drive_right = True
	powersave_right = False
	GPIO.output(PinD_2,GPIO.HIGH)
	while motor_drive_right == True:
		if not (moter_sleep_min < np.abs(dt_moter_right) and np.abs(dt_moter_right) < moter_sleep_max):
			if powersave_right == False:
				powersave_right = True
				GPIO.output(PinC_2,GPIO.HIGH)
			time.sleep(moter_sleep_min)
			continue
		if powersave_right == True:
			powersave_right = False
			GPIO.output(PinC_2,GPIO.LOW)
		if moter_sleep_min < dt_moter_right and dt_moter_right < moter_sleep_max:
			GPIO.output(PinA_2,GPIO.HIGH)
			sleep(np.abs(dt_moter_right))
		if moter_sleep_min < dt_moter_right and dt_moter_right < moter_sleep_max:
			GPIO.output(PinB_2,GPIO.HIGH)
			sleep(np.abs(dt_moter_right))
		if moter_sleep_min < dt_moter_right and dt_moter_right < moter_sleep_max:
			GPIO.output(PinA_2,GPIO.LOW)
			sleep(np.abs(dt_moter_right))
		if moter_sleep_min < dt_moter_right and dt_moter_right < moter_sleep_max:
			GPIO.output(PinB_2,GPIO.LOW)
			sleep(np.abs(dt_moter_right))
		if -moter_sleep_max < dt_moter_right and dt_moter_right < -moter_sleep_min:
			GPIO.output(PinB_2,GPIO.HIGH)
			sleep(np.abs(dt_moter_right))
		if -moter_sleep_max < dt_moter_right and dt_moter_right < -moter_sleep_min:
			GPIO.output(PinA_2,GPIO.HIGH)
			sleep(np.abs(dt_moter_right))
		if -moter_sleep_max < dt_moter_right and dt_moter_right < -moter_sleep_min:
			GPIO.output(PinB_2,GPIO.LOW)
			sleep(np.abs(dt_moter_right))
		if -moter_sleep_max < dt_moter_right and dt_moter_right < -moter_sleep_min:
			GPIO.output(PinA_2,GPIO.LOW)
			sleep(np.abs(dt_moter_right))
	GPIO.output(PinD_2,GPIO.LOW)
def CLEAR_GPIO():
	GPIO.cleanup()
	sleep(0.1)
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
	#		0x08(08)	Bandwidth = 7.31 Hz
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
	#		0x02(02)	ODR = 1000 Hz
	bus.write_byte_data(0x69, 0x10, 0x02)
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

	n = 20
	alpha = 0.1
	beta = 0.1
	theta_zero_judge = 0

	dt = 0.01
	dt_sleep = dt
	t = 0
	t_start = time.time()
	while cal == True:
		for i in range(n):
			#加速度取得と補正
			a_x = (-GET_XACCL() - 0.239738) * 0.980209
			a_z = (GET_ZACCL() + 0.285688) * 0.984507
			#加速度から角度を算出
			theta_ACCL = -np.arctan(a_x/a_z) + 0.002
			v_theta = -GET_YGYRO() + 0.0000822816
			#角速度を積分し角度を算出（補正付き）
			theta_GYRO += v_theta * dt
			#加速度の大きさから角度を加重平均化
			alpha = 0.1 * np.exp(-11.09*np.power(9.80665 - np.sqrt(a_x*a_x+a_z*a_z), 2))
			theta = theta_ACCL * alpha + theta_GYRO * (1 - alpha)
			#ゼロ度検出
			theta_zero_judge = theta_zero_judge * (1 - beta) + np.abs(theta_ACCL) * beta
			if theta_zero_judge < 0.01:
				theta_GYRO = 0
				theta_zero_detection = True
				print("theta zero detected")
			t += dt
			time.sleep(dt_sleep)
		#実時間とプログラム内の時間誤差調整
		dt_sleep = dt - (time.time() - t_start - t) / n
		if dt_sleep < 0.001:
			dt_sleep = 0.001
		if dt < dt_sleep:
			dt_sleep = dt
if __name__ == '__main__':
	try:
		#GPIO設定
		SET_GPIO()

		#i2cとbmx055設定
		SET_BMX055()

		#モーター駆動と角度計算のスレッドスタート
		moter_left = threading.Thread(target=MOTER_LEFT)
		moter_right = threading.Thread(target=MOTER_RIGHT)
		cal_theta = threading.Thread(target=CAL_THETA)
		moter_left.start()
		moter_right.start()
		cal_theta.start()

		#角度ゼロの検出待ち
		print("stand by")
		while theta_zero_detection == False:
			time.sleep(0.01)
		print("theta zero detected")
		print("start control program")

		m = 0.78
		R = 0.025
		I = 0.10617
		r = 0.045

		k_2 = 15
		k_1 = 9.80665 + m * R / 4 / I * k_2 * k_2
		k_3 = 10

		a_x = 0
		v_x = 0.001

		dt = 0.01
		dt_sleep = dt
		t = 0
		t_start = time.time()
		n = 20
		#制御開始
		while True:
			for i in range(n):
				theta_zero_detection == False

				a_x = k_1 * theta + k_2 * v_theta + k_3 * v_x
				v_x += a_x * dt
				
				if np.abs(np.pi / 25 / v_x * r) < moter_sleep_max: #v_xのゼロ割り対策
					dt_moter_left = np.pi / 25 / v_x * r
					dt_moter_right = np.pi / 25 / v_x * r

				t += dt
				time.sleep(dt_sleep)

				#v_xのゼロ検出
				if (powersave_left == True and powersave_right == True) and theta_zero_detection == True:
					v_x = 0.001 #v_xのゼロ割り対策

			#実時間とプログラム内の時間誤差調整
			dt_sleep = dt - (time.time() - t_start - t) / n
			if dt_sleep < 0.001:
				dt_sleep = 0.001
			if dt < dt_sleep:
				dt_sleep = dt

	except KeyboardInterrupt:
		print("\nCtl+C")

	except Exception as e:
		print(str(e))

	finally:
		#スレッド終了
		motor_drive_left = False
		motor_drive_right = False
		cal = False
		moter_left.join()
		moter_right.join()
		cal_theta.join()

		#gpio終了
		CLEAR_GPIO()

		#i2c終了
		bus.close()

		print("\nexit program")