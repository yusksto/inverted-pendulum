#!/usr/bin/python3
# -*- coding: utf-8 -*-

import smbus
import time
from time import sleep
import numpy as np

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

if __name__ == '__main__':
	SET_BMX055()
	f = open('data.dat', 'w')

	
	theta = 0
	theta_GYRO = 0
	theta_ACCL = 0
	v_theta = 0
	
	
	n = 20
	theta_zero_judge = 0

	alpha = 0.95
	beta = 0.9
	gamma = 0.9

	dt = 0.01
	t = 0
	dt_sleep = dt
	t_start = 0
	try:
		t_start = time.time()
		while True:
			for i in range(n):
				#????????????????????????????????????
				a_x = (-GET_XACCL() - 0.239738) * 0.980209
				a_z = (GET_ZACCL() + 0.285688) * 0.984507
				v_theta = v_theta * gamma + (-GET_YGYRO() + 0.0000822816) * (1 - gamma)
				#??????????????????????????????
				theta_ACCL = -np.arctan(a_x/a_z) - 0.010725428306555

				#??????????????????????????????????????????????????????
				theta_GYRO += v_theta * dt

				#???????????????????????????????????????????????????
				theta = (theta + v_theta * dt) * alpha + theta_ACCL * (1 - alpha)

				#???????????????
				theta_zero_judge = theta_zero_judge * beta + np.abs(theta_ACCL) * (1 - beta)
				if theta_zero_judge < 0.01:
					#theta_GYRO = 0
					theta_zero_detection = True

				f.write(str(t) + "	" + str(theta_GYRO) + "	" + str(theta_ACCL) + "	" + str(theta) + "	" + str(v_theta) + "	" + str(dt_sleep) + '\n')

				t += dt
				time.sleep(dt_sleep)
			#???????????????
			dt_sleep = dt - (time.time() - t_start - t) / n
			if dt_sleep < 0.001:
				dt_sleep = 0.001
			if dt < dt_sleep:
				dt_sleep = dt

			print(str(time.time() - t_start))
	except KeyboardInterrupt:
		print("\nCtl+C")
	except Exception as e:
		print(str(e) + '\n')
	finally:
		print(str(time.time()-t_start))
		f.close()
		bus.close()
		print("\nexit program")