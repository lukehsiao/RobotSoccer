#!/usr/bin/python
from numpy import matrix
from numpy import linalg
import math

#define s1,s2,s3

realWorldOffset = 1 #1.698
s = .0282977488817 #radius of wheel
r = .08 #radius from center to center of wheel

r1theta = -math.pi/3.0
r1x = math.cos(r1theta)*r
r1y = math.sin(r1theta)*r
r2theta = math.pi/3.0
r2x = math.cos(r2theta)*r
r2y = math.sin(r2theta)*r
r3theta = math.pi
r3x = math.cos(r3theta)*r
r3y = math.sin(r3theta)*r

#print r1x
#print r1y
#print r2x
#print r2y
#print r3x
#print r3y

s1theta = r1theta - math.pi/2
s1x = math.cos(s1theta)
s1y = math.sin(s1theta)

s2theta = r2theta - math.pi/2
s2x = math.cos(s2theta)
s2y = math.sin(s2theta)

s3theta = r3theta - math.pi/2
s3x = math.cos(s3theta)
s3y = math.sin(s3theta)
#print s1x
#print s1y
#print s2x
#print s2y
#print s3x
#print s3y
mSub = matrix( [[s1x,s1y,(s1y*r1x - s1x*r1y)],
                [s2x,s2y,(s2y*r2x - s2x*r2y)],
                [s3x,s3y,(s3y*r3x - s3x*r3y)]] )
                
#print mSub

M = realWorldOffset*(1.0/s)*mSub

R = lambda theta: matrix( [[math.cos(theta),math.sin(theta),0.0],
             [-math.sin(theta),math.cos(theta),0.0],
             [0.0,0.0,1.0]] )


#print M

def getWheelVel(x,y,omega):
  desired = matrix( [[x],
                     [y],
                     [omega]] )
                   
  result = M*desired

  return result.getA()[0][0], result.getA()[1][0], result.getA()[2][0]
  
def getXYOmega(v1,v2,v3):
  velocity = matrix( [[v1],
                      [v2],
                      [v3]] )
  Minv = linalg.inv(M)
  
  result = Minv*velocity
  
  return result.getA()[0][0], result.getA()[1][0], result.getA()[2][0]

def getRobotXYOmega(x,y,omega,theta):
  desired = matrix( [[x],
                     [y],
                     [omega]] )
  desired = R(theta)*desired
  return desired
  
def getRobotXYOmegaAsTuple(x, y, omega, theta):
  desired = getRobotXYOmega(x, y, omega, theta)
  asArray = desired.getA()
  return asArray[0][0], asArray[1][0], asArray[2][0]

def getWheelVelTheta(x,y,omega,theta):
  desired = getRobotXYOmega(x, y, omega, theta)
                   
  result = M*desired

  return result.getA()[0][0], result.getA()[1][0], result.getA()[2][0]

def radianToQpps(radian):
  result = int(radian * 19820.0 / (2*math.pi))
  #print result 
  if result > 308420:
    return 308420
  elif result < -308420:
    return -308420
  else:
    return result
