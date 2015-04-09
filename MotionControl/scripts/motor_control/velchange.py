#!/usr/bin/python
from roboclaw import *
import math
import mat

DEBUG = True
cap = .3

def radianToQpps(radian):
  result = int(radian * 19820.0 / (2.0*math.pi))
  if result > 308420:
    return 308420
  elif result < -308420:
    return -308420
  else:
    return result
    
def qppsToRadian(qpps):
  result = int(qpps / 19820.0 * (2.0*math.pi))
  return result

def goXYOmega(x,y,omega,limit=False):
  if limit:
    total = math.sqrt(float(x**2+y**2))
    if total > cap:
      scale = cap / total
      x = x * scale
      y = y * scale
  v1,v2,v3 = mat.getWheelVel(x,y,omega)
  s1 = radianToQpps(v1)
  s2 = radianToQpps(v2)
  s3 = radianToQpps(v3)
  SetM1Speed(128,s1)
  SetM2Speed(128,s2)
  SetM1Speed(129,s3)

def goXYOmegaTheta(x,y,omega,theta,limit=False):
  if limit:
    total = math.sqrt(float(x**2+y**2))
    if total > cap:
      scale = cap / total
      x = x * scale
      y = y * scale
  v1,v2,v3 = mat.getWheelVelTheta(x,y,omega,theta)
  s1 = radianToQpps(v1)
  s2 = radianToQpps(v2)
  s3 = radianToQpps(v3)
  SetM1Speed(128,s1)
  SetM2Speed(128,s2)
  SetM1Speed(129,s3)
  
def goXYOmegaAccel(x,y,theta,time=1):
  v1,v2,v3 = mat.getWheelVel(x,y,theta)
  s1 = radianToQpps(v1)
  s2 = radianToQpps(v2)
  s3 = radianToQpps(v3)
  r1 = readM1speed(128)
  r2 = readM2speed(128)
  r3 = readM1speed(129)
  if DEBUG:
    print r1
    print r2
    print r3
  SetM1SpeedAccel(128,int(abs(r1-s1)/time),s1)
  SetM2SpeedAccel(128,int(abs(r2-s2)/time),s2)
  SetM1SpeedAccel(129,int(abs(r3-s3)/time),s3)
  s1_prev=s1
  s2_prev=s2
  s3_prev=s3

def smoothStop(v1_1,v2_1,v3_1,v1_2,v2_2,v3_2):
  s1 = radianToQpps(v1_1)
  s2 = radianToQpps(v2_1)
  s3 = radianToQpps(v3_1)
  SetM1SpeedAccel(128,abs(s1*3),s1)
  SetM2SpeedAccel(128,abs(s2*3),s2)
  SetM1SpeedAccel(129,abs(s3*3),s3)
  time.sleep(.5)
  SetM1SpeedAccel(128,abs(s1*3),0)
  SetM2SpeedAccel(128,abs(s2*3),0)
  SetM1SpeedAccel(129,abs(s3*3),0)
  time.sleep(.5)
  s1 = radianToQpps(v1_2)
  s2 = radianToQpps(v2_2)
  s3 = radianToQpps(v3_2)
  SetM1SpeedAccel(128,abs(s1*3),s1)
  SetM2SpeedAccel(128,abs(s2*3),s2)
  SetM1SpeedAccel(129,abs(s3*3),s3)
  time.sleep(.5)
  SetM1SpeedAccel(128,abs(s1*3),0)
  SetM2SpeedAccel(128,abs(s2*3),0)
  SetM1SpeedAccel(129,abs(s3*3),0)
  time.sleep(.5)
  
def leftTurnMomentum():
  goXYThetaAccel(1.0,0.0,0.0,1)
  time.sleep(2)
  goXYThetaAccel(-3.0,-1.0,0.0,1)
  time.sleep(.25)
  goXYThetaAccel(0.0,-1.0,0.0,.75)
  time.sleep(1)
  goXYThetaAccel(0,0,0,1)


def leftTurn():
  goXYThetaAccel(1.0,0.0,0.0,1)
  time.sleep(2)
  goXYThetaAccel(0.0,-1.0,0.0,1)
  time.sleep(2)
  goXYThetaAccel(0,0,0,1)
  
def printState():
  r1 = readM1speed(128)
  r2 = readM2speed(128)
  r3 = readM1speed(129)
  v1 = qppsToRadian(r1)
  v2 = qppsToRadian(r2)
  v3 = qppsToRadian(r3)
  x,y,omega = mat.getXYOmega(v1,v2,v3)
  print "X=%f" % x
  print "Y=%f" % y
  print "O=%f" % omega

if __name__ == '__main__':
  #smoothStop(v1_1,v2_1,v3_1,v1_2,v2_2,v3_2)
  #smoothStop(v1_3,v2_3,v3_3,v1_4,v2_4,v3_4)
  #goXYThetaAccel(0.0,1.0*2,.5*2,1)
  #time.sleep(1)
  #goXYTheta(0.0,1.0*2,.23*2)
  #time.sleep(3)
  #goXYThetaAccel(0.0,0.0,0.0,1)
  #leftTurn()
  goXYThetaAccel(0,-1,.117907,.5)
  for i in range(0,9):
    time.sleep(.5)
    printState()
  goXYThetaAccel(0,0,0,.5)
#s1 = radianToQpps(v1)
#s2 = radianToQpps(v2)
#s3 = radianToQpps(v3)
#r1 = readM1speed(128)
#r2 = readM2speed(128)
#r3 = readM1speed(129)
#SetM1SpeedAccel(128,int(abs(r1-s1)),s1)
#SetM2SpeedAccel(128,int(abs(r2-s2)),s2)
#SetM1SpeedAccel(129,int(abs(r3-s3)),s3)
#time.sleep(1)
#
#v1,v2,v3 = mat.getWheelVel(0.0,-1.0,0.0)
#s1 = radianToQpps(v1)
#s2 = radianToQpps(v2)
#s3 = radianToQpps(v3)
#r1 = readM1speed(128)
#r2 = readM2speed(128)
#r3 = readM1speed(129)
#SetM1SpeedAccel(128,int(abs(r1-s1)),s1)
#SetM2SpeedAccel(128,int(abs(r2-s2)),s2)
#SetM1SpeedAccel(129,int(abs(r3-s3)),s3)
#time.sleep(1)
#
#v1,v2,v3 = mat.getWheelVel(-1.0,0.0,0.0)
#s1 = radianToQpps(v1)
#s2 = radianToQpps(v2)
#s3 = radianToQpps(v3)
#r1 = readM1speed(128)
#r2 = readM2speed(128)
#r3 = readM1speed(129)
#SetM1SpeedAccel(128,int(abs(r1-s1)),s1)
#SetM2SpeedAccel(128,int(abs(r2-s2)),s2)
#SetM1SpeedAccel(129,int(abs(r2-s3)),s3)
#time.sleep(1)
#
#v1,v2,v3 = mat.getWheelVel(0.0,1.0,0.0)
#s1 = radianToQpps(v1)
#s2 = radianToQpps(v2)
#s3 = radianToQpps(v3)
#r1 = readM1speed(128)
#r2 = readM2speed(128)
#r3 = readM1speed(129)
#SetM1SpeedAccel(128,int(abs(r1-s1)),s1)
#SetM2SpeedAccel(128,int(abs(r2-s2)),s2)
#SetM1SpeedAccel(129,int(abs(r3-s3)),s3)
#time.sleep(1)
#
#r1 = readM1speed(128)
#r2 = readM2speed(128)
#r3 = readM1speed(129)
#SetM1SpeedAccel(128,int(abs(r1)),0)
#SetM2SpeedAccel(128,int(abs(r2)),0)
#SetM1SpeedAccel(129,int(abs(r3)),0)
#
#time.sleep(1)

#while True:
#	time.sleep(1)
#	print "M1"
#	print readM1speed(128)
#	print "M2"
#	print readM2speed(128)
