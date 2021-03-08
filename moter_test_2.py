
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
from time import sleep
class TB6674PG:
    def __init__(self, PinA, PinB, PinC):
        self.mPinA = PinA
        self.mPinB = PinB
        self.mPinC = PinC
        self.mStep = 0
        self.SetWaitTime(0.01)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.mPinA, GPIO.OUT)
        GPIO.setup(self.mPinB, GPIO.OUT)
        GPIO.setup(self.mPinC, GPIO.OUT)
    def SetWaitTime(self, wait):
        if wait < 0.01:
            self.mStep_wait = 0.005
        elif wait > 0.5:
            self.mStep_wait = 0.1
        else:
            self.mStep_wait = wait
    def Step_CW(self):
        GPIO.output(self.mPinA,GPIO.HIGH)
        sleep(self.mStep_wait)
        GPIO.output(self.mPinB,GPIO.HIGH)
        sleep(self.mStep_wait)
        GPIO.output(self.mPinA,GPIO.LOW)
        sleep(self.mStep_wait)
        GPIO.output(self.mPinB,GPIO.LOW)
        sleep(self.mStep_wait)
    def Step_CCW(self):
        GPIO.output(self.mPinB,GPIO.HIGH)
        sleep(self.mStep_wait)
        GPIO.output(self.mPinA,GPIO.HIGH)
        sleep(self.mStep_wait)
        GPIO.output(self.mPinB,GPIO.LOW)
        sleep(self.mStep_wait)
        GPIO.output(self.mPinA,GPIO.LOW)
        sleep(self.mStep_wait)
    def SetPosition(self, step, duration):
        diff_step = step - self.mStep
        if diff_step != 0:
            wait = abs(float(duration)/float(diff_step)/4)
            print("duration:"+str(duration))
            print("diff_step:"+str(diff_step))
            print("wait:"+str(wait))
            self.SetWaitTime(wait)
        GPIO.output(self.mPinC,GPIO.HIGH)
        for i in range(abs(diff_step)):
            if diff_step > 0:
                self.Step_CW()
            if diff_step < 0:
                self.Step_CCW()
        self.mStep = step
        GPIO.output(self.mPinC,GPIO.LOW)
    def Cleanup(self):
        GPIO.cleanup()
if __name__ == '__main__':
    StepMoter1 = TB6674PG(PinA=6,PinB=13,PinC=26)
    StepMoter2 = TB6674PG(PinA=10,PinB=22,PinC=11)

    #Main loop
    try:
        while True:
            StepMoter1.SetPosition(0,2)
            StepMoter2.SetPosition(0,2)
            sleep(0.5)
            StepMoter1.SetPosition(25,1)
            StepMoter2.SetPosition(25,1)
            sleep(0.5)
            StepMoter1.SetPosition(50,2)
            StepMoter2.SetPosition(50,2)
            sleep(0.5)
            StepMoter1.SetPosition(75,3)
            StepMoter2.SetPosition(75,3)
            sleep(0.5)
    except KeyboardInterrupt:
        print("\nCtl+C")
    except Exception as e:
        print(str(e))
    finally:
        StepMoter1.Cleanup()
        StepMoter2.Cleanup()
        print("\nexit program")