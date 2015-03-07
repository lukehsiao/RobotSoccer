#!/usr/bin/env python

import rospy
import sys
#from robot_soccer.srv import *

class Strategy:
    def __init__(self):
        self.home1_x = 0
        self.home1_y = 0
        self.home1_theta = 0.0
        self.isBehindBall = False
    
    def run(self):
        while(True):
            while(not self.isBehindBall):
                self.update_locations()
                self.skill_go_to_behind_ball()
            while(self.isBehindBall):
                self.update_locations()
                self.skill_rush_goal()
        #while robot is not behind ball
            #skill_go_to_behind_ball()
        
    #comment out later (replace with luna's)
    def skill_go_to_behind_ball(self):
        print "moving behind ball"
        print "got behind ball"
        self.isBehindBall = True
    
    #comment out later (replace with luna's)
    def skill_rush_goal(self):
        print "rushing goal"
        print "lost ball"
        self.isBehindBall = False
    
    def update_locations(self):
        print "updating locations"
        #rospy.wait_for_service('service_name')
        try:
            #locations = rospy.ServiceProxy('service_name', ServiceName)
            #response = locations()
            self.home1_x = self.home1_x + 1
            self.home1_y = self.home1_y + 1
            self.home1_theta = self.home1_theta + 0.1
            print 'x: %d  y: %d  theta: %f' %(self.home1_x, self.home1_y, self.home1_theta)
        except rospy.ServiceException, e:
            print "service call failed: %s"%e

if __name__ == '__main__':
    s = Strategy()
    s.run()
    
    
    
    
    
    
    
    
    
    
    
    
