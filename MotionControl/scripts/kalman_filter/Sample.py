import math
from param import *

class Sample:
    def __init__(self):
        self.time = 0
        self.home1_x = 0.0
        self.home1_y = 0.0
        self.home1_theta = 0.0
        self.home2_x = 0.0
        self.home2_y = 0.0
        self.home2_theta = 0.0
        self.away1_x = 0.0
        self.away1_y = 0.0
        self.away1_theta = 0.0
        self.away2_x = 0.0
        self.away2_y = 0.0
        self.away2_theta = 0.0
        self.ball_x = 0.0
        self.ball_y = 0.0
        self.kill = 0.0
        
    def setDataFromSample(self,data):
        self.time = round(timeToInt(data.header.stamp),2)
        self.home1_theta = round(degreeToRadian(data.home1_theta),3)
        home1_x = pixelToMeter(data.home1_x)
        home1_y = pixelToMeter(data.home1_y)
        angleField = math.atan2(home1_y, home1_x)
        mag = math.sqrt(home1_x**2+home1_y**2)
        angleCamera = math.atan(HEIGHT_CAMERA/mag)
        offset = HEIGHT_ROBOT / math.tan(angleCamera)
        home1_x = home1_x - offset * math.cos(angleField)
        home1_y = home1_y - offset * math.sin(angleField)       
        self.home1_x = round(home1_x,3)
        self.home1_y = round(home1_y,3)
        self.home2_x = pixelToMeter(data.home2_x)
        self.home2_y = pixelToMeter(data.home2_y)
        self.home2_theta = degreeToRadian(data.home2_theta)
        self.away1_x = pixelToMeter(data.away1_x)
        self.away1_y = pixelToMeter(data.away1_y)
        self.away1_theta = degreeToRadian(data.away1_theta)
        self.away2_x = pixelToMeter(data.away2_x)
        self.away2_y = pixelToMeter(data.away2_y)
        self.away2_theta = degreeToRadian(data.away2_theta)
        self.ball_x = pixelToMeter(data.ball_x)
        self.ball_y = pixelToMeter(data.ball_y)      
        
    def getDiscreteSample(self):
        home1_x = meterToPixel(self.home1_x);
        home1_y = meterToPixel(self.home1_y);
        home1_theta = radianToDegree(self.home1_theta);
        home2_x = meterToPixel(self.home2_x);
        home2_y = meterToPixel(self.home2_y);
        home2_theta = radianToDegree(self.home2_theta);
        away1_x = meterToPixel(self.away1_x);
        away1_y = meterToPixel(self.away1_y);
        away1_theta = radianToDegree(self.away1_theta);
        away2_x = meterToPixel(self.away2_x);
        away2_y = meterToPixel(self.away2_y);
        away2_theta = radianToDegree(self.away2_theta);
        ball_x = meterToPixel(self.ball_x);
        ball_y = meterToPixel(self.ball_y);
        
        return (home1_x, home1_y, home1_theta,
        home2_x, home2_y, home2_theta,
        away1_x, away1_y, away1_theta,
        away2_x, away2_x, away2_theta,
        ball_x, ball_y)
        
