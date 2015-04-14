#!/usr/bin/env python

from collections import deque
import time
from motor_control import velchange
from motor_control import mat
import math
import copy
from param import *

class RobotLocation:
  def __init__(self,timestamp = 0.0, x = 0.0, y = 0.0, theta = 0.0):
    self.timestamp = timestamp
    self.x = x
    self.y = y
    self.theta = theta

class RobotCommand():
  def __init__(self, timestamp = -1,
               vel_x = 0.0, vel_y = 0.0, omega = 0.0, theta = 0.0):
    if theta != 0.0:
      vel_x, vel_y, omega = mat.getRobotXYOmegaAsTuple(vel_x, vel_y, omega,
                                                       theta)
    self.timestamp = timestamp
    self.vel_x = vel_x
    self.vel_y = vel_y
    self.omega = omega
    #optimization could be added here
    self.mag = math.sqrt(vel_x**2+vel_y**2)
    self.angle = math.atan2(vel_y,vel_x)
    self.expiration = 0
  
  def execute(self):
    velchange.goXYOmega(self.vel_x, self.vel_y, self.omega)
    self.timestamp = getTime()
  
  def getCommandToSend(self):
    return (self.timestamp, self.vel_x, self.vel_y, self.omega)
  
  def isKill(self):
    return self.vel_x == 0 and self.vel_y == 0 and self.omega == 0

class HomeError(Exception):
   def __init__(self, value):
     self.value = value
   def __str__(self):
     return repr(self.value)

class HomeRobot:
  def __init__(self):
    self.isInitialized = False
    self.init_sample = 0
    self.isBusy = False
    self.previousLocation = RobotLocation()
    self.commandQueue = deque()
    self.upToLastCommandCache = None
    self.realTimeCache = None
  
  TIMESTEP = .01
  
  def pendBusy(self):
    while self.isBusy:
      pass
    self.isBusy = True;
    
  def postBusy(self):
    self.isBusy = False;
    
  def getLocationFromHistory(self, stopTime = -1):
    location = copy.deepcopy(self.previousLocation)
    done = False
    
    for command in self.commandQueue:
      while location.timestamp < command.expiration:
        motionAngle = location.theta + command.angle
        location.x = location.x + command.mag*math.cos(motionAngle)*self.TIMESTEP
        location.y = location.y + command.mag*math.sin(motionAngle)*self.TIMESTEP
        location.theta = (location.theta + command.omega*self.TIMESTEP)%(2*math.pi)
        location.timestamp = location.timestamp + 1
        lastCommand = command
        if stopTime != -1 and location.timestamp == stopTime:
          print "stop time reached"
          done = True
          break
      if done:
        break
        
    return location
  
  def getLocation(self,timestamp = -1):
    if timestamp <= 0:
      timestamp = getTime()
    self.pendBusy()
    
    # case where request is before our known history
    if timestamp < self.previousLocation.timestamp:
      location = copy.deepcopy(self.previousLocation)
      self.postBusy()
      #nothing has changed from this timestamp and the beginning of history.
      location.timestamp = timestamp
      return location
#      raise HomeError("Request for location before beginning of history")
    
    # case where timestamp is between beginning of history and first command.
    if len(self.commandQueue) == 0 or self.commandQueue[0].timestamp >= timestamp:
      location = copy.deepcopy(self.previousLocation)
      self.postBusy()
      #nothing has changed from this timestamp and the beginning of history.
      location.timestamp = timestamp
      return location
    
    # case request is before the last command
    if self.commandQueue[-1].timestamp > timestamp:
      location = getLocationFromHistory(timestamp)
      self.postBusy()
      return location
    
    # case timestamp is >= self.commandQueue[-1].timestamp
    
    # updating cache
    if self.upToLastCommandCache == None:
      # expire real time cache
      self.realTimeCache = None
      # update cache
      self.upToLastCommandCache = self.getLocationFromHistory(timestamp)
    
    #cache selection
    if self.realTimeCache != None and self.realTimeCache.timestamp <= timestamp:
      location = copy.deepcopy(self.realTimeCache)
    else:
      location = copy.deepcopy(self.upToLastCommandCache)
    
    # propagation of last command until requested time
    command = self.commandQueue[-1]
    while location.timestamp < timestamp:
      # could increment theta by half a timestep here for more accurate modeling
      motionAngle = location.theta + command.angle
      location.x = location.x + command.mag*math.cos(motionAngle)*self.TIMESTEP
      location.y = location.y + command.mag*math.sin(motionAngle)*self.TIMESTEP
      location.theta = (location.theta + command.omega*self.TIMESTEP)%(2*math.pi)
      location.timestamp = location.timestamp + 1
    self.realTimeCache = copy.deepcopy(location)
    self.postBusy()
    return location
    
  def addCommand(self,command):
    self.pendBusy()
    
    # case where command is before beginning of history
    if command.timestamp < self.previousLocation.timestamp:
      if len(self.commandQueue) != 0:
        self.postBusy()
        return 
        #raise HomeError('Recieved command out of order.')
    
    # case of kill command being sent; throw out if command queue is empty or
    # if last command sent was a kill command too
    if command.isKill() and (len(self.commandQueue) == 0 or self.commandQueue[-1].isKill()):
      self.postBusy()
      return

    if len(self.commandQueue) == 0:
      # for a single command on the queue do not make a cache.
      self.upToLastCommandCache = None
      self.commandQueue.append(command)
      self.postBusy()
      return
    else:
      # for multiple commands: get cache updated to new command,
      # set expiration on previous command
      self.postBusy() # TODO: THIS IS A SCREAMING SEMAPHOR VIOLATION!!!
      location = self.getLocation(command.timestamp)
      self.pendBusy()
      self.predictedLocation = location
      self.commandQueue[-1].expiration = command.timestamp
      self.commandQueue.append(command)
      self.postBusy()

  def updateLocation(self, newLocation):
    self.pendBusy()
    
    # case tried to update to before the beginning of history
    if newLocation.timestamp < self.previousLocation.timestamp:
      self.postBusy()
      raise HomeError('Tried to update with location before beginning of history')
    
    # clear caches
    self.upToLastCommandCache = None
    self.realTimeCache = None
    
    # remove expired commands.
    while len(self.commandQueue) > 0:
      commandIsKill = self.commandQueue[0].isKill()
      expiration = self.commandQueue[0].expiration
      if commandIsKill or (expiration > 0 and expiration <= newLocation.timestamp):
        self.commandQueue.popleft()
      else:
        break

    # update previous location
    self.previousLocation = newLocation
        
    self.postBusy()
      
