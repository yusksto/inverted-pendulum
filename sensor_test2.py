
#!/usr/bin/python3
# -*- coding: utf-8 -*-

import smbus
import time
import numpy as np

# Get I2C bus
bus = smbus.SMBus(1)


# BMX055 Accl address, 0x19(24)
# Select PMU_Range register, 0x0F(15)
#		0x03(03)	Range = +/- 2g
bus.write_byte_data(0x19, 0x0F, 0x03)
# BMX055 Accl address, 0x19(24)
# Select PMU_BW register, 0x10(16)
#		0x08(08)	Bandwidth = 1000 Hz
bus.write_byte_data(0x19, 0x10, 0x0F)
# BMX055 Accl address, 0x19(24)
# Select PMU_LPW register, 0x11(17)
#		0x00(00)	Normal mode, Sleep duration = 0.5ms
bus.write_byte_data(0x19, 0x11, 0x00)


# BMX055 Gyro address, 0x69(104)
# Select Range register, 0x0F(15)
#		0x04(04)	Full scale = +/- 500 degree/s
bus.write_byte_data(0x69, 0x0F, 0x02)
# BMX055 Gyro address, 0x69(104)
# Select Bandwidth register, 0x10(16)
#		0x07(07)	ODR = 1000 Hz
bus.write_byte_data(0x69, 0x10, 0x02)
# BMX055 Gyro address, 0x69(104)
# Select LPM1 register, 0x11(17)
#		0x00(00)	Normal mode, Sleep duration = 2ms
bus.write_byte_data(0x69, 0x11, 0x00)


time.sleep(0.5)

xAccl = 0
yAccl = 0
zAccl = 0

xGyro = 0
yGyro = 0
zGyro = 0

def GET_ACCL():
	# BMX055 Accl address, 0x19(24)
	# Read data back from 0x02(02), 6 bytes
	# xAccl LSB, xAccl MSB, yAccl LSB, yAccl MSB, zAccl LSB, zAccl MSB
	data = bus.read_i2c_block_data(0x19, 0x02, 6)

	global xAccl
	global yAccl
	global zAccl

	# Convert the data to 12-bits
	xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16
	if xAccl > 2047 :
		xAccl -= 4096
	yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16
	if yAccl > 2047 :
		yAccl -= 4096
	zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16
	if zAccl > 2047 :
		zAccl -= 4096
	xAccl *= 0.0098
	yAccl *= 0.0098
	zAccl *= 0.0098

def GET_GYRO():
	# BMX055 Gyro address, 0x69(104)
	# Read data back from 0x02(02), 6 bytes
	# xGyro LSB, xGyro MSB, yGyro LSB, yGyro MSB, zGyro LSB, zGyro MSB
	data = bus.read_i2c_block_data(0x69, 0x02, 6)

	global xGyro
	global yGyro
	global zGyro

	# Convert the data
	xGyro = data[1] * 256 + data[0]
	if xGyro > 32767 :
		xGyro -= 65536
	yGyro = data[3] * 256 + data[2]
	if yGyro > 32767 :
		yGyro -= 65536
	zGyro = data[5] * 256 + data[4]
	if zGyro > 32767 :
		zGyro -= 65536
	xGyro *= 0.0153
	yGyro *= 0.0153
	zGyro *= 0.0153


if __name__ == '__main__':
	f = open('data.dat', 'w')
	dt = 0.01
	t = 0
	while t < 10:
		GET_ACCL()
		GET_GYRO()
		f.write(str(t) + " " + str(yGyro) + '\n')
		t += dt
		time.sleep(dt)
	f.close()