#!/usr/bin/env python
import rospy
from motor_control.roboclaw import *
import math
import time
from std_msgs.msg import String
from robot_soccer.msg import *
from robot_soccer.srv import *
from motor_control import mat
from motor_control import velchange
from kalman_filter.Locations import Locations
from gamepieces.HomeRobot import *
from kalman_filter.Ball import *
from collections import deque
import param
import cPickle as pickle
import copy

# Global Variables for initialization
class PredictionNode:
  def __init__(self):
    self.lastLocations = Locations()
    self.home1 = HomeRobot()
    self.sampling = False

  def pendSamplingSemaphore(self):
    while self.sampling:
      pass
    self.sampling = True

  def postSamplingSemaphore(self):
    self.sampling = False
  

  def callback(self,data):
      # parse message
    measuredLocations = Locations()
    measuredLocations.setLocationsFromMeasurement(data)
    self.pendSamplingSemaphore()
    self.lastLocations = measuredLocations
    self.home1.updateLocation(self.lastLocations.home1)   
    self.postSamplingSemaphore()

  def getPredictionsData(self,req):
      return predictionsResponse(*(self.lastSample.getDiscreteSample()))

  def getLocations(self,req):
      newLocations = copy.deepcopy(self.lastLocations)
      newLocations.home1 = self.home1.getLocation()     
      return curlocsResponse(pickle.dumps(newLocations))

#  def getBallLoc(self,req):
#      return balllocResponse(getTime(), self.ball.point.x, self.ball.point.y)

  def recieveCommand(self,req):
      command = RobotCommand(req.timestamp, req.vel_x, req.vel_y, req.omega)
      self.home1.addCommand(command)
      return commandsentResponse()
  
  def server(self):
      rospy.Service('predictionsService', predictions, self.getPredictionsData)
      rospy.Service('locations', curlocs, self.getLocations)
#      rospy.Service('ballloc', ballloc, self.getBallLoc)
      rospy.Service('commandSent', commandsent,self.recieveCommand)

  def listener(self):
      rospy.init_node('kalmanFilterNode')
      # This subscribes to the velTopic topic expecting the 'velocities' message
      rospy.Subscriber("locTopic", locations, self.callback)

  def run(self):
    print "Welcome to the motion prediction node:\nProgram starting"
    self.listener()
    print "Starting services." 
    self.server()
    i = 0
    rospy.spin()

if __name__ == '__main__':
  p = PredictionNode()
  p.run()
