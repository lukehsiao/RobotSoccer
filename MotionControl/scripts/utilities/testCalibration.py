#!/usr/bin/python
from motor_control import velchange
import math
import time

scale = .3
velchange.goXYOmega(scale,0,0)
time.sleep(1)
velchange.goXYOmega(0,0,0)
time.sleep(1)
velchange.goXYOmega(-scale,0,0)
time.sleep(1)
velchange.goXYOmega(0,0,0)
time.sleep(1)
velchange.goXYOmega(scale*math.cos(2*math.pi/3),scale*math.sin(2*math.pi/3),0)
time.sleep(1)
velchange.goXYOmega(0,0,0)
time.sleep(1)
velchange.goXYOmega(-math.cos(2*math.pi/3)*scale,-math.sin(2*math.pi/3)*scale,0)
time.sleep(1)
velchange.goXYOmega(0,0,0)
time.sleep(1)
velchange.goXYOmega(math.cos(-2*math.pi/3)*scale,math.sin(-2*math.pi/3)*scale,0)
time.sleep(1)
velchange.goXYOmega(0,0,0)
time.sleep(1)
velchange.goXYOmega(-math.cos(-2*math.pi/3)*scale,-math.sin(-2*math.pi/3)*scale,0)
time.sleep(1)
velchange.goXYOmega(0,0,0)
