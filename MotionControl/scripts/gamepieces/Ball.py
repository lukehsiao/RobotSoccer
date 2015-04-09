from MotionSkills import *

class BallLocation(self):
  def __init__(self, timestamp = 0, x = 0.0, y = 0.0, magnitude = 0.0, angle = 0.0):
    self.timestamp = timestamp
    self.x = x
    self.y = y
    self.magnitude = magnitude
    self.angle = angle
    

class Ball:
    def __init__(self):
        self.point = Point()
        self.init_sample = 0
        self.time = 0.0
        self.isInitialized = False
