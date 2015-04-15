#!/usr/bin/env python

# This file is meant to be run from the vision computer, not on the Odroid.

from param import *
from Tkinter import *
import math
import rospy
from robot_soccer.srv import *
import cPickle as pickle

REFRESH_RATE = 100

class Field(Frame):
    def sendCommandCenter(self,command):
        sendCommCenter = rospy.ServiceProxy('commcenter', commcenter)
        try:
          resp1 = sendCommCenter(command)
        except rospy.ServiceException as exc:
          print("Service did not process request: " + str(exc))

    def key(self,event):
        keyPressed = event.char
        if keyPressed == 'g': # Run the "Play" strategy
          self.sendCommandCenter(2)
        elif keyPressed == 's': # Stop
          self.sendCommandCenter(1)
        elif keyPressed == 'c': # Go To Center
          self.sendCommandCenter(3)
        elif keyPressed == 'p': # Go to starting position
          self.sendCommandCenter(4)
        elif keyPressed == 't': # Run the "Test" strategy
          self.sendCommandCenter(5)
        print "pressed", repr(event.char)

    def callback(self,event):
        self.focus_set()
        print "clicked at", event.x, event.y
  
    def __init__(self, parent):
        Frame.__init__(self, parent)   
        self.ball = 0
        self.ball_x = 0
        self.ball_y = 0
        self.home = 0
        self.home_x = 0
        self.home_y = 0
        self.home_theta = 0
        self.away = 0
        self.away_x = 0
        self.away_y = 0
        self.away_theta = 0
         
        self.parent = parent        
        self.initUI()
    
    def getBallCoord(self):
      return (GUI_CENTER_X + self.ball_x - WIDTH_BALL/2,
              GUI_CENTER_Y - self.ball_y - WIDTH_BALL/2,         
              GUI_CENTER_X + self.ball_x + WIDTH_BALL/2,
              GUI_CENTER_Y - self.ball_y + WIDTH_BALL/2)

    def getHomeCoord(self):
      return (GUI_CENTER_X + self.home_x + int(math.cos(math.pi/6+self.home_theta)*RADIUS_ROBOT),
              GUI_CENTER_Y - self.home_y - int(math.sin(math.pi/6+self.home_theta)*RADIUS_ROBOT),
              GUI_CENTER_X + self.home_x + int(math.cos(math.pi/2+self.home_theta)*RADIUS_ROBOT),
              GUI_CENTER_Y - self.home_y - int(math.sin(math.pi/2+self.home_theta)*RADIUS_ROBOT),
              GUI_CENTER_X + self.home_x + int(math.cos(math.pi*5/6+self.home_theta)*RADIUS_ROBOT),
              GUI_CENTER_Y - self.home_y - int(math.sin(math.pi*5/6+self.home_theta)*RADIUS_ROBOT),
              GUI_CENTER_X + self.home_x + int(math.cos(math.pi*7/6+self.home_theta)*RADIUS_ROBOT),
              GUI_CENTER_Y - self.home_y - int(math.sin(math.pi*7/6+self.home_theta)*RADIUS_ROBOT),
              GUI_CENTER_X + self.home_x + int(math.cos(math.pi*3/2+self.home_theta)*RADIUS_ROBOT),
              GUI_CENTER_Y - self.home_y - int(math.sin(math.pi*3/2+self.home_theta)*RADIUS_ROBOT),
              GUI_CENTER_X + self.home_x + int(math.cos(math.pi*11/6+self.home_theta)*RADIUS_ROBOT),
              GUI_CENTER_Y - self.home_y - int(math.sin(math.pi*11/6+self.home_theta)*RADIUS_ROBOT),
              GUI_CENTER_X + self.home_x,
              GUI_CENTER_Y - self.home_y)
              
    def getAwayCoord(self):
      return (GUI_CENTER_X + self.away_x + int(math.cos(math.pi/6+self.away_theta)*RADIUS_ROBOT),
              GUI_CENTER_Y - self.away_y - int(math.sin(math.pi/6+self.away_theta)*RADIUS_ROBOT),
              GUI_CENTER_X + self.away_x + int(math.cos(math.pi/2+self.away_theta)*RADIUS_ROBOT),
              GUI_CENTER_Y - self.away_y - int(math.sin(math.pi/2+self.away_theta)*RADIUS_ROBOT),
              GUI_CENTER_X + self.away_x + int(math.cos(math.pi*5/6+self.away_theta)*RADIUS_ROBOT),
              GUI_CENTER_Y - self.away_y - int(math.sin(math.pi*5/6+self.away_theta)*RADIUS_ROBOT),
              GUI_CENTER_X + self.away_x + int(math.cos(math.pi*7/6+self.away_theta)*RADIUS_ROBOT),
              GUI_CENTER_Y - self.away_y - int(math.sin(math.pi*7/6+self.away_theta)*RADIUS_ROBOT),
              GUI_CENTER_X + self.away_x + int(math.cos(math.pi*3/2+self.away_theta)*RADIUS_ROBOT),
              GUI_CENTER_Y - self.away_y - int(math.sin(math.pi*3/2+self.away_theta)*RADIUS_ROBOT),
              GUI_CENTER_X + self.away_x + int(math.cos(math.pi*11/6+self.away_theta)*RADIUS_ROBOT),
              GUI_CENTER_Y - self.away_y - int(math.sin(math.pi*11/6+self.away_theta)*RADIUS_ROBOT),
              GUI_CENTER_X + self.away_x,
              GUI_CENTER_Y - self.away_y)    
   
    
    def updateLocations(self):
      try:
          locations = rospy.ServiceProxy('locations', curlocs)
          response = locations()
          locations = pickle.loads(response.pickle)
          self.ball_x = meterToPixel(locations.ball.x)
          self.ball_y = meterToPixel(locations.ball.y)
          self.canvas.coords(self.ball,*self.getBallCoord())
          self.home_x = meterToPixel(locations.home1.x)
          self.home_y = meterToPixel(locations.home1.y)
          self.home_theta = locations.home1.theta
          self.canvas.coords(self.home,*self.getHomeCoord())
          self.away_x = meterToPixel(locations.away1.x)
          self.away_y = meterToPixel(locations.away1.y)
          self.away_theta = locations.away1.theta
          self.canvas.coords(self.away,*self.getAwayCoord())
          
      except rospy.ServiceException, e:
          print "service call failed: %s"%e
      self.parent.after(REFRESH_RATE, self.updateLocations)
        
    def initUI(self):
      
        self.parent.title("Colors")
        self.parent.bind("<Key>", self.key)
        self.parent.bind("<Button-1>", self.callback)
        self.pack(fill=BOTH, expand=1)
        self.canvas = Canvas(self)        
        #field border
        self.canvas.create_rectangle(GUI_MARGIN, GUI_MARGIN, WIDTH_FIELD + GUI_MARGIN, HEIGHT_FIELD + GUI_MARGIN)
        #center line
        self.canvas.create_line(WIDTH_FIELD/2 + GUI_MARGIN, GUI_MARGIN, WIDTH_FIELD/2 + GUI_MARGIN, HEIGHT_FIELD + GUI_MARGIN)
        #home goal
        self.canvas.create_line(GUI_MARGIN, GUI_MARGIN + (HEIGHT_FIELD - HEIGHT_GOAL)/2,
            GUI_MARGIN + WIDTH_GOAL, GUI_MARGIN + (HEIGHT_FIELD - HEIGHT_GOAL)/2,
            GUI_MARGIN + WIDTH_GOAL, GUI_MARGIN + (HEIGHT_FIELD - HEIGHT_GOAL)/2 + HEIGHT_GOAL,
            GUI_MARGIN, GUI_MARGIN + (HEIGHT_FIELD - HEIGHT_GOAL)/2 + HEIGHT_GOAL)
        #away goal
        self.canvas.create_line(GUI_MARGIN + WIDTH_FIELD, GUI_MARGIN + (HEIGHT_FIELD - HEIGHT_GOAL)/2,
            GUI_MARGIN + WIDTH_FIELD - WIDTH_GOAL, GUI_MARGIN + (HEIGHT_FIELD - HEIGHT_GOAL)/2,
            GUI_MARGIN + WIDTH_FIELD - WIDTH_GOAL, GUI_MARGIN + (HEIGHT_FIELD - HEIGHT_GOAL)/2 + HEIGHT_GOAL,
            GUI_MARGIN + WIDTH_FIELD, GUI_MARGIN + (HEIGHT_FIELD - HEIGHT_GOAL)/2 + HEIGHT_GOAL)        
        #center circle
        self.canvas.create_oval(GUI_MARGIN + WIDTH_FIELD/2 - HEIGHT_GOAL/2, GUI_MARGIN + HEIGHT_FIELD/2 - HEIGHT_GOAL/2,
            GUI_MARGIN + WIDTH_FIELD/2 + HEIGHT_GOAL/2, GUI_MARGIN + HEIGHT_FIELD/2 + HEIGHT_GOAL/2)
        #canvas.create_rectangle(150, 10, 240, 80, 
        #    outline="#f50", fill="#f50")
        #canvas.create_rectangle(270, 10, 370, 80, 
        #    outline="#05f", fill="#05f")
        self.ball = self.canvas.create_oval(*self.getBallCoord(), fill='yellow')
        self.home = self.canvas.create_polygon(*self.getHomeCoord(), fill = 'blue')
        self.away = self.canvas.create_polygon(*self.getAwayCoord(), fill = 'red')
        self.canvas.pack(fill=BOTH, expand=1)


def main():
  print "Welcome to the Vektor Command Center!"
  print "Searching for Vektor Location Service..."
  rospy.wait_for_service('locations')
  print "Searching for Vektor Gameplay Command Service..."
  rospy.wait_for_service('commcenter')
  print "Starting Command Center"
  root = Tk()
  root.resizable(width=FALSE, height=FALSE)
  ex = Field(root)
  root.after(0, ex.updateLocations)
  root.geometry("{}x{}".format(WIDTH_FIELD+2*GUI_MARGIN, HEIGHT_FIELD+2*GUI_MARGIN))
  root.mainloop()
  
if __name__ == '__main__':
    main()
