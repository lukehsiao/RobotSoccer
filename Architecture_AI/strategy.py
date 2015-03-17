#!/usr/bin/env python

import param
import math
import motor_control
import rospy
import sys
from robot_soccer.srv import *

class Strategy:
    def __init__(self):
        self.home1_x = 0
        self.home1_y = 0
        self.home1_theta = 0.0
        self.isBehindBall = False
        self.ball_x = 0
        self.ball_y = 0
        self.isFacingGoal = False

    def run(self):
        while(True):
            while(not self.isBehindBall):
                self.update_locations()
                motor_control.skill_go_to_behind_ball(self)
            '''
            while(self.isBehindBall and not self.isFacingGoal):
                self.update_locations()
                motor_control.skill_face_goal(self)
            '''
            while(self.isBehindBall):
                self.update_locations()
                motor_control.skill_rush_goal(self)

    def updateIsBehindBall(self):
        theta_btw_ball_N_goal = math.atan2(param.goal_y - self.ball_y, param.goal_x - self.ball_x)
        behind_x = self.ball_x - 75*math.cos(theta_btw_ball_N_goal)
        behind_y = self.ball_y - 75*math.sin(theta_btw_ball_N_goal)
        """
        behind_x = self.ball_x - 50
        behind_y = self.ball_y + (self.ball_y/5)
        """
        dist = math.sqrt((behind_x - self.home1_x)**2 + (behind_y - self.home1_y)**2)
        y_dist = abs(behind_y - self.home1_y)
        print "behind_y: %d  behind_x: %d  y_dist: %d" %(behind_y, behind_x, y_dist)
        if(dist < 50 and abs(behind_y - self.home1_y) < 30):
            print "robot is behind ball, dist: %.2f" % dist
            self.isBehindBall = True
        else:
            print "robot is not behind ball, dist: %.2f" % dist
            self.isBehindBall = False

    def update_locations(self):
        print "updating locations"
        rospy.wait_for_service('predictionsService')
        try:
            locations = rospy.ServiceProxy('predictionsService', predictions)
            response = locations()
            self.home1_x = response.home1_x
            self.home1_y = response.home1_y
            self.home1_theta = response.home1_theta
            self.ball_x = response.ball_x
            self.ball_y = response.ball_y
            self.updateIsBehindBall()
            print 'x: %d  y: %d  theta: %f' %(self.home1_x, self.home1_y, self.home1_theta)
            print 'ball x: %d  ball y: %d' %(self.ball_x, self.ball_y)
        except rospy.ServiceException, e:
            print "service call failed: %s"%e

if __name__ == '__main__':
    s = Strategy()
    s.run()














