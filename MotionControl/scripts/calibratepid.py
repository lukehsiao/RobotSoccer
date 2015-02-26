#!/usr/bin/python
from roboclaw import *

p = int(65536 * 4) #262144
i = int(65536 * 2) #131072
d = int(65536 * 6)  #65536
q = 308419

print readmainbattery()
p1,i1,d1,q1 = readM1pidq(128)
print "128 M1 P=%.2f" % (p1/65536.0)
print "128 M1 I=%.2f" % (i1/65536.0)
print "128 M1 D=%.2f" % (d1/65536.0)
print "128 M1 QPPS=",q1
p2,i2,d2,q2 = readM2pidq(128)
print "128 M2 P=%.2f" % (p2/65536.0)
print "128 M2 I=%.2f" % (i2/65536.0)
print "128 M2 D=%.2f" % (d2/65536.0)
print "128 M2 QPPS=",q2
p3,i3,d3,q3 = readM1pidq(129)
print "121 M1 P=%.2f" % (p3/65536.0)
print "129 M1 I=%.2f" % (i3/65536.0)
print "129 M1 D=%.2f" % (d3/65536.0)
print "129 M1 QPPS=",q3

M1Forward(128,127);
M2Forward(128,127);
M1Forward(129,127);
time.sleep(2)

newq1=0
newq2=0
newq3=0

count = 3
for x in range(0,count):
  newq1 = newq1 + readM1speed(128)
  newq2 = newq2 + readM2speed(128)
  newq3 = newq3 + readM1speed(129)
  time.sleep(.5)
 
M1Forward(128,0);
M2Forward(128,0);
M1Forward(129,0);

newq1 = newq1 / count
newq2 = newq2 / count
newq3 = newq3 / count

SetM1pidq(128,p,i,d,newq1)
SetM2pidq(128,p,i,d,newq2)
SetM1pidq(129,p,i,d,newq3)

p1,i1,d1,q1 = readM1pidq(128)
print "128 M1 P=%.2f" % (p1/65536.0)
print "128 M1 I=%.2f" % (i1/65536.0)
print "128 M1 D=%.2f" % (d1/65536.0)
print "128 M1 QPPS=",q1
p2,i2,d2,q2 = readM2pidq(128)
print "128 M2 P=%.2f" % (p2/65536.0)
print "128 M2 I=%.2f" % (i2/65536.0)
print "128 M2 D=%.2f" % (d2/65536.0)
print "128 M2 QPPS=",q2
p3,i3,d3,q3 = readM1pidq(129)
print "121 M1 P=%.2f" % (p3/65536.0)
print "129 M1 I=%.2f" % (i3/65536.0)
print "129 M1 D=%.2f" % (d3/65536.0)
print "129 M1 QPPS=",q3

sendcommand(128,94)
sendcommand(129,94)
