#!/usr/bin/env python

import math
import rospy
import sys
import time
from robot_soccer.srv import *
from MotionSkills import *
from motor_control import roboclaw
from gamepieces.HomeRobot import HomeRobot
from gamepieces.HomeRobot import RobotCommand
from kalman_filter.Ball import *
from param import *
from utilities.kick import kick
from kalman_filter.Locations import *
import sched
import cPickle as pickle

from enum import Enum
class GameState(Enum):
  stop = 1
  play = 2
  center = 3
  startPosition = 4
  test = 5

class State(Enum):
  rushGoal = 1
  getBehindBall = 2
  rotateToAngleBehindBall = 3
  check = 4
  returnToPlay = 5

class TestState(Enum):
  check = 1
  rushGoal = 2
  getBehindBall = 3
  
class Rotate(Enum):
  none = 1
  clockwise = 2
  counterClockwise = 3

class Vektory:
  def __init__(self):
    self.locations = None
    self.ball = Ball()
    self.robotLocation = None
    self.distanceToBall = 0
    self.state = State.check
    self.rotate = Rotate.none
    self.stopRushingGoalTime = 0
    self.newCommand = False
    self.vel_x = 0
    self.vel_y = 0
    self.omega = 0
    self.gameState = GameState.stop
    self.stopped = True
    self.testState = TestState.check

  def sendCommand(self, vel_x, vel_y, omega, theta = 0):
    command = RobotCommand(-1,vel_x, vel_y, omega, theta)
    command.execute()
    sendCommand = rospy.ServiceProxy('commandSent', commandsent)
    try:
      resp1 = sendCommand(*(command.getCommandToSend()))
    except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))

  def rotateAroundBallToAngle(self,targetAngle):
    #lookAtPoint = self.ball.point
    delta = MotionSkills.deltaBetweenAngles(self.robotLocation.theta, targetAngle)    
    vektor_x = 0
    vektor_y = 0
    if abs(delta) < .1:
      self.state = State.check
      return
    if delta > 0:
      vektor_y = -CIRCLE_SPEED
    else:
      vektor_y = CIRCLE_SPEED
    
    #angle = math.atan2(lookAtPoint.y-self.robotLocation.y, lookAtPoint.x-self.robotLocation.x)
    
    #delta_angle = angle-self.robotLocation.theta
    
    #bestDelta = math.atan2(math.sin(delta_angle), math.cos(delta_angle)) * SCALE_OMEGA
    #print bestDelta
      
    #if bestDelta < MIN_DELTA and bestDelta > -MIN_DELTA:
    #  bestDelta = 0
    good = CIRCLE_SPEED/self.distanceToBall
    good_y = 0
    if delta > 0:
      good_y = -CIRCLE_SPEED
    else:
      good_y = CIRCLE_SPEED
    self.sendCommand(0, good_y, good, 0)
    #self.sendCommand(vektor_x, vektor_y, bestDelta, 0)

  def go_direction(self, point):
    angle = MotionSkills.angleBetweenPoints(self.robotLocation,point)
    self.vel_x = math.cos(angle) * MAX_SPEED
    self.vel_y = math.sin(angle) * MAX_SPEED
    des_angle = MotionSkills.angleBetweenPoints(self.ball.point,HOME_GOAL)
    delta_angle = MotionSkills.deltaBetweenAngles(self.robotLocation.theta,des_angle)
    if abs(delta_angle) < .1:
      self.omega = 0
    else:
      self.omega = delta_angle
    self.newCommand = True

  def go_direction2(self, point):
    angle = MotionSkills.angleBetweenPoints(self.robotLocation,point)
    vel_x = math.cos(angle) * MAX_SPEED
    vel_y = math.sin(angle) * MAX_SPEED
    des_angle = MotionSkills.angleBetweenPoints(self.ball.point,HOME_GOAL)
    delta_angle = MotionSkills.deltaBetweenAngles(self.robotLocation.theta,des_angle)
    if abs(delta_angle) < .1:
      omega = 0
    else:
      omega = delta_angle * 3.0
    self.sendCommand(vel_x,vel_y,omega,self.robotLocation.theta)
    
  
  def go_to_point(self,x, y, lookAtPoint=None):
    #print "go_to_point"
    if lookAtPoint == None:
      lookAtPoint = self.ball.point
    desired_x = x
    desired_y = y
    
    vektor_x = (desired_x-self.robotLocation.x) * SCALE_VEL
    vektor_y = (desired_y-self.robotLocation.y) * SCALE_VEL
    
    mag = math.sqrt(vektor_x**2+vektor_y**2) 
    angle = math.atan2(lookAtPoint.y-self.robotLocation.y, lookAtPoint.x-self.robotLocation.x)
    
    delta_angle = angle-self.robotLocation.theta
    
    bestDelta = math.atan2(math.sin(delta_angle), math.cos(delta_angle)) * SCALE_OMEGA
    #print bestDelta
    if mag >= MAX_SPEED:
      vektor_x = (MAX_SPEED/mag)*vektor_x
      vektor_y = (MAX_SPEED/mag)*vektor_y
    elif mag < MIN_SPEED:
      vektor_x = 0
      vektor_y = 0
      
    if bestDelta < MIN_DELTA and bestDelta > -MIN_DELTA:
      bestDelta = 0
    self.sendCommand(vektor_x, vektor_y, bestDelta, self.robotLocation.theta)
    
  def updateLocations(self):
    try:
        locations = rospy.ServiceProxy('locations', curlocs)
        response = locations()
        self.locations = pickle.loads(response.pickle)
        #print (self.locations.ball.x, self.locations.ball.y)
        self.robotLocation = self.locations.home1
        self.ball.point.x = self.locations.ball.x
        self.ball.point.y = self.locations.ball.y
        self.distanceToBall = math.sqrt((self.robotLocation.x-self.locations.ball.x)**2+(self.robotLocation.y-self.locations.ball.y)**2)
        #print 'time: %f x: %f  y: %f  theta: %f' %(robotLocation.timestamp, robotLocation.x, robotLocation.y, robotLocation.theta)
    except rospy.ServiceException, e:
        print "service call failed: %s"%e

  def commandRoboclaws(self):
    velchange.goXYOmegaTheta(self.vel_x,self.vel_y,self.omega,self.robotLocation.theta)
  
  def old(self):
    self.updateLocations()
    if self.testState == TestState.check:
      if self.robotLocation.x > (self.ball.point.x-DIS_BEHIND_BALL):
        self.testState = TestState.getBehindBall
      else:
        self.testState = TestState.rushGoal
        self.stopRushingGoalTime = getTime() + 100
        
    if self.testState == TestState.getBehindBall:
      point = Point(self.ball.point.x-DIS_BEHIND_BALL, self.ball.point.y)
      if self.robotLocation.y > self.ball.point.y:
        point.y = point.y + DIS_BEHIND_BALL
      else:
        point.y = point.y - DIS_BEHIND_BALL
      self.go_direction2(point)
      self.testState = TestState.check
      
    if self.testState == TestState.rushGoal:
      if self.ball.point.y < -.25 and self.robotLocation.y > (self.ball.point.y + DIS_BEHIND_BALL):
        point = Point(self.ball.point.x,self.ball.point.y - 1.0)
        self.go_direction2(point)
      elif self.ball.point.y > .25 and self.robotLocation.y < (self.ball.point.y - DIS_BEHIND_BALL):
        point = Point(self.ball.point.x,self.ball.point.y + 1.0)
        self.go_direction2(point)      
      elif abs(self.distanceToBall) > (DIS_BEHIND_BALL*3/4):
        self.go_direction2(self.ball.point)
      else:
        self.go_direction2(HOME_GOAL)
      if getTime() >= self.stopRushingGoalTime:
        self.testState = TestState.check
        
  def test(self):
    global HOME_GOAL
    self.updateLocations()
    HOME_GOAL = Point(HOME_GOAL.x,0)
    
    # Kick the ball off of the sides when it is too close to the side
    if self.ball.point.y < (MARGIN - HEIGHT_FIELD_METER/2):
      HOME_GOAL = Point(HOME_GOAL.x, -HEIGHT_FIELD_METER)
    if self.ball.point.y > (HEIGHT_FIELD_METER/2 - MARGIN):
      HOME_GOAL = Point(HOME_GOAL.x, HEIGHT_FIELD_METER)
      
    # Make sure the robot is behind the ball and facing the goal  
    if self.testState == TestState.check:
      self.testState = TestState.getBehindBall
      angleBallGoal = MotionSkills.angleBetweenPoints(self.ball.point,HOME_GOAL)
      deltaAngle = MotionSkills.deltaBetweenAngles(self.robotLocation.theta,angleBallGoal)
      if MotionSkills.isPointInFrontOfRobot(self.robotLocation,self.ball.point) and abs(deltaAngle) < .12:
        self.testState = TestState.rushGoal
        self.stopRushingGoalTime = getTime() + 45
    
    # Get behind the ball. Move to the side first is necessary    
    if self.testState == TestState.getBehindBall:
      if (self.ball.point.x + .05) < (self.robotLocation.x):
          point = Point(self.ball.point.x)
          if self.ball.point.y < self.robotLocation.y:
              point.y = self.ball.point.y + DIS_BEHIND_BALL
          else:
              point.y = self.ball.point.y - DIS_BEHIND_BALL
          #Move to the side of the ball
          self.go_to_point(point.x,point.y)
      #Go to behind ball
      else:
          behindTheBallPoint = MotionSkills.getPointBehindBall(self.ball,HOME_GOAL)
          self.go_to_point(behindTheBallPoint.x,behindTheBallPoint.y)
      self.testState = TestState.check
    
    # Rush the goal and kick for the specified time  
    if self.testState == TestState.rushGoal:
      self.go_to_point(HOME_GOAL.x,HOME_GOAL.y,HOME_GOAL)
      if self.distanceToBall < .05:
        kick()
      if self.stopRushingGoalTime <= getTime():
        self.testState = TestState.check
  
  def play(self):
    self.updateLocations()
    self.commandRoboclaws()
    #print (self.robotLocation.x, self.robotLocation.y)
    #self.setSpeed()
    #check if robot is ready to rush goal
    if self.state == State.check:
      self.state = State.getBehindBall 
      if self.robotLocation.x > pixelToMeter(345):
        self.state = State.returnToPlay
      elif (MotionSkills.isPointInFrontOfRobot(self.robotLocation,self.ball.point, 0.5, 0.04 + abs(MAX_SPEED/4))): #This offset compensates for the momentum
        self.state = State.rushGoal# rush goal
        self.stopRushingGoalTime = getTime() + int(2 * DIS_BEHIND_BALL/MAX_SPEED*100)  

    if self.state == State.rushGoal:
      #self.speed = RUSH_SPEED
      #self.go_to_point(HOME_GOAL.x, HOME_GOAL.y, HOME_GOAL)
      self.go_direction(HOME_GOAL)
      if getTime() >= self.stopRushingGoalTime:
        kick()
        self.state = State.check
    
    if self.state == State.returnToPlay:
      self.go_to_point(CENTER.x, CENTER.y, HOME_GOAL)
      if abs(self.robotLocation.x) < .2 and abs(self.robotLocation.y) < .2:
        self.state = State.check
      
    #check if ball is behind robot
    if self.state == State.getBehindBall:
      #robot in front of ball
      if MotionSkills.isBallBehindRobot(self.robotLocation, self.ball.point):
        # This gets a point beside the ball perpendicular to the line of the ball and the goal
        #point = getPointBesideBall(self.robotLocation, self.ball.point, DIS_BEHIND_BALL)
        
        point = Point(self.ball.point.x)
        # if robot above ball
        if self.ball.point.y < self.robotLocation.y:
            point.y = self.ball.point.y + DIS_BEHIND_BALL
        else:
            point.y = self.ball.point.y - DIS_BEHIND_BALL

        self.go_direction(point)
        #Move to the side of the ball
        #self.go_to_point(point.x,point.y)
      #robot behind ball
      else:
        behindTheBallPoint = MotionSkills.getPointBehindBall(self.ball)  
        self.go_direction(behindTheBallPoint)
        self.state = State.check  
  
  def executionLoop(self, scheduler):
    scheduler.enter(.05, 1, self.executionLoop,(scheduler,))
    if self.gameState == GameState.play:
      self.play()
    elif self.gameState == GameState.test:
      self.test()
    elif self.gameState == GameState.stop:
      if self.stopped == False:
        self.sendCommand(0,0,0);
        self.stopped = True;
    elif self.gameState == GameState.center:
      self.updateLocations()
      if abs(self.robotLocation.x) > .1 or abs(self.robotLocation.y) > .1 or abs(self.robotLocation.theta) > .1:
        self.go_to_point(CENTER.x, CENTER.y, HOME_GOAL)
      elif self.stopped == False:
        self.sendCommand(0,0,0);
        self.stopped = True;
    elif self.gameState == GameState.startPosition:
      self.updateLocations()
      if abs(self.robotLocation.x - pixelToMeter(-120)) > .1 or abs(self.robotLocation.y - 0) > .1 or abs(self.robotLocation.theta) > .1:
        self.go_to_point(pixelToMeter(-115), 0, HOME_GOAL)
      elif self.stopped == False:
        self.sendCommand(0,0,0);
        self.stopped = True;      

  
  def executeCommCenterCommand(self,req):
    if req.comm == 1:
      self.gameState = GameState.stop
      self.state = State.check
      self.stopped = False
    elif req.comm == 2:
      self.gameState = GameState.play
    elif req.comm == 3:
      self.gameState = GameState.center
      self.state = State.check
      self.stopped = False
    elif req.comm == 4:
      self.gameState = GameState.startPosition
      self.state = State.check
      self.stopped = False
    elif req.comm == 5:
      self.testState = TestState.check
      self.gameState = GameState.test

    return commcenterResponse()
  
  def go(self):
    roboclaw.calibrateRoboclaws()
    print "Searching for Vektor Locations Service..."
    rospy.wait_for_service('locations')
    print "Searching for Vektor Locations Update Service..."
    rospy.wait_for_service('commandSent')
    print "Entering Gameplay"
    rospy.init_node('commandNode')
    rospy.Service('commcenter', commcenter, self.executeCommCenterCommand)
    s = sched.scheduler(time.time, time.sleep)
    s.enter(0,1,self.executionLoop,(s,))
    s.run()

if __name__ == '__main__':
  winner = Vektory()
  winner.go()
