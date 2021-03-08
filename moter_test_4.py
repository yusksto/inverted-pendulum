
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
from time import sleep
import smbus
import time
import threading
import numpy as np
step = 200
a = 3.1415 * 2 / step
dt_1 = 0
PinA_1=6
PinB_1=13
PinC_1=26
PinA_2=10
PinB_2=22
PinC_2=11
d=0
GPIO.setmode(GPIO.BCM)
GPIO.setup(PinA_1, GPIO.OUT)
GPIO.setup(PinB_1, GPIO.OUT)
GPIO.setup(PinC_1, GPIO.OUT)
GPIO.setup(PinA_2, GPIO.OUT)
GPIO.setup(PinB_2, GPIO.OUT)
GPIO.setup(PinC_2, GPIO.OUT)
def Step_CW_1():
    GPIO.output(PinC_1,GPIO.HIGH)
    for i in range(100):
        GPIO.output(PinA_1,GPIO.HIGH)
        sleep(dt_1)
        GPIO.output(PinB_1,GPIO.HIGH)
        sleep(dt_1)
        GPIO.output(PinA_1,GPIO.LOW)
        sleep(dt_1)
        GPIO.output(PinB_1,GPIO.LOW)
        sleep(dt_1)
    GPIO.output(PinC_1,GPIO.LOW)
def Step_CCW_1():
    GPIO.output(PinC_1,GPIO.HIGH)
    while d == 0:
        GPIO.output(PinB_1,GPIO.HIGH)
        sleep(dt_1)
        GPIO.output(PinA_1,GPIO.HIGH)
        sleep(dt_1)
        GPIO.output(PinB_1,GPIO.LOW)
        sleep(dt_1)
        GPIO.output(PinA_1,GPIO.LOW)
        sleep(dt_1)
    GPIO.output(PinC_1,GPIO.LOW)
def Step_CW_2():
    GPIO.output(PinC_2,GPIO.HIGH)
    while d == 0:
        GPIO.output(PinA_2,GPIO.HIGH)
        sleep(dt_1)
        GPIO.output(PinB_2,GPIO.HIGH)
        sleep(dt_1)
        GPIO.output(PinA_2,GPIO.LOW)
        sleep(dt_1)
        GPIO.output(PinB_2,GPIO.LOW)
        sleep(dt_1)
    GPIO.output(PinC_2,GPIO.LOW)
def Step_CCW_2():
    GPIO.output(PinC_2,GPIO.HIGH)
    while d == 0:
        GPIO.output(PinB_2,GPIO.HIGH)
        sleep(dt_1)
        GPIO.output(PinA_2,GPIO.HIGH)
        sleep(dt_1)
        GPIO.output(PinB_2,GPIO.LOW)
        sleep(dt_1)
        GPIO.output(PinA_2,GPIO.LOW)
        sleep(dt_1)
    GPIO.output(PinC_2,GPIO.LOW)
if __name__ == '__main__':
    b = 1
    c = 0.1
    dt_1 = a / b / 4
    t1 = threading.Thread(target=Step_CW_1)
    t1.start()
    try:
        while True:
            dt = a / b / 4
            sleep(dt)
    except KeyboardInterrupt:
        print("\nCtl+C")
    except Exception as e:
        print(str(e))
    finally:
        d=1
        sleep(0.1)
        GPIO.cleanup()
        print("\nexit program")