#!/usr/bin/env python
# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
from time import sleep
import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)


# BMX055 Accl address, 0x19(24)
# Select PMU_Range register, 0x0F(15)
#		0x03(03)	Range = +/- 2g
bus.write_byte_data(0x19, 0x0F, 0x03)
# BMX055 Accl address, 0x19(24)
# Select PMU_BW register, 0x10(16)
#		0x08(08)	Bandwidth = 7.81 Hz
bus.write_byte_data(0x19, 0x10, 0x08)
# BMX055 Accl address, 0x19(24)
# Select PMU_LPW register, 0x11(17)
#		0x00(00)	Normal mode, Sleep duration = 0.5ms
bus.write_byte_data(0x19, 0x11, 0x00)


# BMX055 Gyro address, 0x69(104)
# Select Range register, 0x0F(15)
#		0x04(04)	Full scale = +/- 125 degree/s
bus.write_byte_data(0x69, 0x0F, 0x04)
# BMX055 Gyro address, 0x69(104)
# Select Bandwidth register, 0x10(16)
#		0x07(07)	ODR = 100 Hz
bus.write_byte_data(0x69, 0x10, 0x07)
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
	xGyro *= 0.0038
	yGyro *= 0.0038
	zGyro *= 0.0038

#変数
step = 200
a = 3.1415 * 2 / step
PinA_1=6
PinB_1=13
PinC_1=26
PinA_2=10
PinB_2=22
PinC_2=11
GPIO.setmode(GPIO.BCM)
GPIO.setup(PinA_1, GPIO.OUT)
GPIO.setup(PinB_1, GPIO.OUT)
GPIO.setup(PinC_1, GPIO.OUT)
GPIO.setup(PinA_2, GPIO.OUT)
GPIO.setup(PinB_2, GPIO.OUT)
GPIO.setup(PinC_2, GPIO.OUT)
def Step_CW(dt_1):
    GPIO.output(PinC_1,GPIO.HIGH)
    GPIO.output(PinC_2,GPIO.HIGH)
    GPIO.output(PinA_1,GPIO.HIGH)
    GPIO.output(PinA_2,GPIO.HIGH)
    sleep(dt_1)
    GPIO.output(PinB_1,GPIO.HIGH)
    GPIO.output(PinB_2,GPIO.HIGH)
    sleep(dt_1)
    GPIO.output(PinA_1,GPIO.LOW)
    GPIO.output(PinA_2,GPIO.LOW)
    sleep(dt_1)
    GPIO.output(PinB_1,GPIO.LOW)
    GPIO.output(PinB_2,GPIO.LOW)
    sleep(dt_1)
    GPIO.output(PinC_1,GPIO.LOW)
    GPIO.output(PinC_2,GPIO.LOW)
def Step_CCW(dt_1):
    GPIO.output(PinC_1,GPIO.HIGH)
    GPIO.output(PinC_2,GPIO.HIGH)
    GPIO.output(PinB_1,GPIO.HIGH)
    GPIO.output(PinB_2,GPIO.HIGH)
    sleep(dt_1)
    GPIO.output(PinA_1,GPIO.HIGH)
    GPIO.output(PinA_2,GPIO.HIGH)
    sleep(dt_1)
    GPIO.output(PinB_1,GPIO.LOW)
    GPIO.output(PinB_2,GPIO.LOW)
    sleep(dt_1)
    GPIO.output(PinA_1,GPIO.LOW)
    GPIO.output(PinA_2,GPIO.LOW)
    sleep(dt_1)
    GPIO.output(PinC_1,GPIO.LOW)
    GPIO.output(PinC_2,GPIO.LOW)
if __name__ == '__main__':
    b = 0.1
    c = 0.1
    try:
        while True:
            dt = a / b / 4
            if 0.01 < dt:
                if dt < 0.5:
                    Step_CW(dt)
            b += c * dt
    except KeyboardInterrupt:
        print("\nCtl+C")
    except Exception as e:
        print(str(e))
    finally:
        GPIO.cleanup()
        print("\nexit program")