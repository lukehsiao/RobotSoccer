#!/usr/bin/env python
import rospy
from roboclaw import *
import math
from std_msgs.msg import String
from robot_soccer.msg import velocities
from robot_soccer.msg import locations
import mat
import param
import velchange
#241p/meter

# Global Variables for initialization
sampling = False
init_sample_num = 5
init_sample = 0
init_msg_time = 0
init_odroid_time = 0
previous_update_time = 0
home1_initialized = True

home1_vx = 0
home1_vy = 0
home1_omega = 0

home1_ry = 0
home1_rx = 0
home1_theta = 0

home1_ry_d = 0
home1_rx_d = 0
home1_theta_d = 0

home1_Qx = .1
home1_Qy = .1
home1_Qtheta = .1

home1_Sx = 1.0
home1_Sy = 1.0
home1_Stheta = 1.0

home1_Rx = .015
home1_Ry = .015
home1_Rtheta = .09

home1_Lx = 0.0
home1_Ly = 0.0
home1_Ltheta = 0.0

timeToFloat = lambda x: x.secs + x.nsecs/1000000000.0
pixelToMeter = lambda x: x/241.0
degreeToRadian = lambda x: x/180.0 * math.pi

def initialze_home1():
    init_sample = 0

def pendSamplingSemaphore():
    global sampling
    while sampling:
      pass
    sampling = True

def postSamplingSemaphore():
    global sampling
    sampling = False
    
def update_home1(current_time = time.time()):
    global home1_rx
    global home1_ry
    global home1_theta
    global home1_rx_d
    global home1_ry_d
    global home1_theta_d
    global home1_vx
    global home1_vy
    global home1_omega
    global home1_Qx
    global home1_Qy
    global home1_Qtheta
    global home1_Sx
    global home1_Sy
    global home1_Stheta
    global previous_update_time
    delta_time = (current_time - previous_update_time)
    home1_rx = home1_rx + delta_time*home1_vx
    home1_ry = home1_ry + delta_time*home1_vy
    home1_theta = home1_theta + delta_time*home1_omega
    home1_rx_d = home1_rx_d + delta_time*home1_vx
    home1_ry_d = home1_ry_d + delta_time*home1_vy
    home1_theta_d = home1_theta_d + delta_time*home1_omega
    home1_Sx = home1_Sx + delta_time*home1_Qx
    home1_Sy = home1_Sy + delta_time*home1_Qy
    home1_Stheta = home1_Stheta + delta_time*home1_Qtheta

    previous_update_time = current_time
    
    
def callback(data):
    # parse message
    odroid_time = time.time()
    msg_time = timeToFloat(data.header.stamp);
    msg_home1_x = pixelToMeter(data.home1_x);
    msg_home1_y = pixelToMeter(data.home1_y);
    msg_home1_theta = degreeToRadian(data.home1_theta);
    
    #print odroid_time
    #print msg_time
    #print home1_x
    #print home1_y
    #print home1_theta
    
    global sampling
    global init_sample
    global init_sample_num
    global init_msg_time
    global init_odroid_time
    global previous_update_time
    global home1_rx
    global home1_ry
    global home1_theta
    global home1_rx_d
    global home1_ry_d
    global home1_theta_d
    global home1_initialized
    global home1_Sx
    global home1_Sy
    global home1_Stheta
    global home1_Rx
    global home1_Ry
    global home1_Rtheta
    global home1_Lx
    global home1_Ly
    global home1_Ltheta
    
    
    pendSamplingSemaphore()
    if home1_initialized == False:
      home1_rx = ((home1_rx * init_sample) + msg_home1_x)/float(init_sample + 1)
      home1_rx_d = home1_rx
      home1_ry = ((home1_ry * init_sample) + msg_home1_y)/float(init_sample + 1)
      home1_ry_d = home1_ry
      home1_theta = ((home1_theta * init_sample) + msg_home1_theta)/float(init_sample + 1)
      print "home1_theta %f" % home1_theta
      home1_theta_d = home1_theta
      print home1_theta_d
      init_sample = init_sample + 1
      init_msg_time = msg_time
      init_odroid_time = odroid_time
      previous_update_time = odroid_time
      if init_sample == init_sample_num:
        init_sample = 0
        print "Init of home1_theta_d %f" % home1_theta_d 
        home1_initialized = True
    else:
      home1_Lx = home1_Sx/(home1_Rx+home1_Sx)
      home1_Ly = home1_Sy/(home1_Ry+home1_Sy)
      home1_Ltheta = home1_Stheta/(home1_Rtheta+home1_Stheta)
      
      home1_Sx = (1.0 - home1_Lx)*home1_Sx 
      home1_Sy = (1.0 - home1_Ly)*home1_Sy 
      home1_Stheta = (1.0 - home1_Ltheta)*home1_Stheta 
      
      update_home1(odroid_time) 
      home1_rx = home1_rx + home1_Lx*(msg_home1_x - home1_rx)
      home1_ry = home1_ry + home1_Ly*(msg_home1_y - home1_ry)
      home1_theta = home1_theta + home1_Ltheta*(msg_home1_theta - home1_theta)
      
    postSamplingSemaphore()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node('kalman_filter', anonymous=True)

    # This subscribes to the velTopic topic expecting the 'velocities' message
    rospy.Subscriber("locTopic", locations, callback)


if __name__ == '__main__':
    listener()
    
    init_sample = 0
    pendSamplingSemaphore()
    home1_initialized = False
    postSamplingSemaphore()
    
    while home1_initialized == False:
      time.sleep(1)
    
    print "Initial home1_rx = %f" % home1_rx
    print "Initial home1_ry = %f" % home1_ry
    print "Initial home1_theta = %f" % home1_theta
    print "Initial init_msg_time = %f" % init_msg_time
    print "Initial init_odroid_time = %f" % init_odroid_time

    si = 2.0
    
    pendSamplingSemaphore()
    update_home1()
    home1_vy = .1
    velX = home1_vx + si * (home1_rx - home1_rx_d)
    velY = home1_vy + si * (home1_ry - home1_ry_d)
    velTheta = home1_omega + si * (home1_theta - home1_theta_d)
    print "home1_vx: %f" % home1_vx
    print "home1_rx: %f" % home1_rx
    print "home1_rx_d: %f" % home1_rx_d
    print velX
    print velY
    print "home1_omega: %f" % home1_omega
    print "home1_theta: %f" % home1_theta
    print "home1_theta_d: %f" % home1_theta_d
    print velTheta
    velchange.goXYTheta(velX, velY, velTheta)
    postSamplingSemaphore()

    while True:
      time.sleep(.05)
      pendSamplingSemaphore()
      update_home1()
      velX = home1_vx + si * (home1_rx - home1_rx_d)
      velY = home1_vy + si * (home1_ry - home1_ry_d)
      velTheta = home1_omega + si * (home1_theta - home1_theta_d)
      velchange.goXYTheta(velX, velY, velTheta)
      postSamplingSemaphore()