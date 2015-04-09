#!/usr/bin/env python
from gamepieces.HomeRobot import *
from gamepieces.Ball import *
import cPickle as pickle


class LocationFilter:
  def __init__(self):
    self.lastSample = None

  def callback(self,data):
    # parse message
    measuredLocations = Locations()
    measuredLocations.setDataFromSample(data)
    measuredRobotLocation = RobotLocation(measuredSample.time, measuredSample.home1_x, 
                           newSample.home1_y, newSample.home1_theta)
    newBallLocation = BallLocation(newSample.time, newSample.ball_x, newSample.ball_y)
    
    updatedRobotLocation = self.filterRobotLocation(newRobotLocation)
    self.ball.time = newSample.time
      self.ball.point.x = newSample.ball_x
      self.ball.point.y = newSample.ball_y
    self.lastSample = newSample
    self.postSamplingSemaphore()
