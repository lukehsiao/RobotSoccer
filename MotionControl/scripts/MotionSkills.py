import time
from motor_control import velchange
import math
from numpy import matrix
from kalman_filter.Sample import *
from param import *
from Point import Point

class Command:
    def __init__(self, vel_x = 0.0, vel_y = 0.0, omega = 0.0, theta = 0.0, runTime = 0.0):
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.omega = omega
        self.theta = theta
        self.runTime = runTime
        self.start = 0.0
        self.stop = 0.0
    
    def setCommand(self, timestamp, vel_x, vel_y, omega):
        self.start = timestamp
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.omega = omega 
    
    def getCommandToSend(self):
        return self.start, self.vel_x, self.vel_y, self.omega
    
    def run(self):
        self.start = round(time.time(),2)
        velchange.goXYOmegaTheta(self.vel_x, self.vel_y, self.omega, self.theta)
    
    def launch(self):
        self.start = round(time.time(),2)
        self.stop = round(self.start+self.runTime,2)
        velchange.goXYOmegaTheta(self.vel_x, self.vel_y, self.omega, self.theta)
        while(time.time() < self.stop):
            pass
        velchange.goXYOmega(0,0,0)
        
class MotionSkills:
    def __init__(self):
        pass
        
    @staticmethod
    def go_to_point(currentRobotState, desiredPoint, percentage = 0):
        length = math.sqrt((desiredPoint.x-currentRobotState.pos_x_est)**2+(desiredPoint.y-currentRobotState.pos_y_est)**2)
        if percentage != 0:
          length = length * percentage
        angle = math.atan2(desiredPoint.y-currentRobotState.pos_y_est,desiredPoint.x-currentRobotState.pos_x_est)
        runTime = length/SPEED_ROBOT
        vel_x = math.cos(angle)*SPEED_ROBOT
        vel_y = math.sin(angle)*SPEED_ROBOT
        return Command(vel_x, vel_y, 0, currentRobotState.pos_theta_est, runTime)
    
    @staticmethod
    def disBetweenPoints(point1,point2):
      return math.sqrt((point2.y-point1.y)**2+(point2.x-point1.x)**2)

    @staticmethod    
    def angleBetweenPoints(firstPoint,secondPoint):
        angle = math.atan2(secondPoint.y-firstPoint.y,secondPoint.x-firstPoint.x)
        # print "desired angle: %f" % radianToDegree(angle)
        return angle
        
    @staticmethod
    def deltaBetweenAngles(first,second):
        delta_angle = second - first
        #if delta_angle < -math.pi:
         #   delta_angle = delta_angle + (math.pi * 2)
        #elif delta_angle > math.pi:
        #    delta_angle = delta_angle - (math.pi * 2)
        # print "delta: %f" % radianToDegree(delta_angle)
        return math.atan2(math.sin(delta_angle), math.cos(delta_angle))
    
    @staticmethod
    def go_to_angle(currentRobotState, lookToPoint):
        currentAngle = currentRobotState.pos_theta_est
        # print "current angle: %f" % currentAngle
        point = Point(currentRobotState.pos_x_est,currentRobotState.pos_y_est)
        desiredAngle = MotionSkills.angleBetweenPoints(point, lookToPoint)
        delta =  MotionSkills.deltaBetweenAngles(currentAngle, desiredAngle)
        runTime = abs(delta/SPEED_ROTATION)
    
        speed = SPEED_ROTATION
    
        if delta < 0:
            speed = -speed

        return Command(0, 0, speed, 0, runTime)
    
    @staticmethod
    def getPointBehindBall(ball, home_goal = None):
        if home_goal == None:
          home_goal = HOME_GOAL
        ballPoint = ball.point
        angle = MotionSkills.angleBetweenPoints(home_goal, ballPoint)
        x = ballPoint.x + (DIS_BEHIND_BALL * math.cos(angle))
        y = ballPoint.y + (DIS_BEHIND_BALL * math.sin(angle))
        #print "(%f, %f)" % (x, y)
        return Point(x,y)
        
    @staticmethod
    def rotatePointByAngle(point,angle):
        R = matrix([[math.cos(angle),math.sin(angle)],
                    [-math.sin(angle),math.cos(angle)]])
        pointMat = matrix([[point.x],[point.y]])
        rotated = R * pointMat
        array = rotated.getA()
        return Point(round(array[0][0],3),round(array[1][0],3))

    @staticmethod
    #behind_x is the allowed distance behind the ball the robot can go while still rushing the ball
    def isPointInFrontOfRobot(robotLoc,point,param_x = .5, param_y = .04):
        refPoint = Point(point.x-robotLoc.x,point.y - robotLoc.y)
        rotatedPoint = MotionSkills.rotatePointByAngle(refPoint, robotLoc.theta)
        
        if rotatedPoint.x > 0 and rotatedPoint.x < param_x and rotatedPoint.y < param_y and rotatedPoint.y > -param_y:
            return True
        return False
        
    @staticmethod
    # This method finds the point "distFromBall" away from the ball perpendicular to the line from the ball to the goal
    def getPointBesideBall(robotLoc, ball, distFromBall):
      angleBallToGoal = getAngleBetweenPoints(ball, HOME_GOAL)
      angleToDesiredPoint = 0
      if(angleBallToGoal > 0):
        angleToDesiredPoint = angleBallToGoal + math.pi/2
      else:
        angleToDesiredPoint = angleBallToGoal - math.pi/2
      beside_x = ball.x - (distFromBall * math.cos(angleToDesiredPoint))
      if ball.y > 0:
        beside_y = ball.y - (distFromBall * math.sin(angleToDesiredPoint))
      else:
        beside_y = ball.y + (distFromBall * math.sin(angleToDesiredPoint))
      pointBesideBall = Point(beside_x, beside_y)
      return pointBesideBall
        
    @staticmethod
    def isBallBehindRobot(robotLoc, ball):
      distRobot_x = (HOME_GOAL.x - robotLoc.x)
      distRobot_y = (HOME_GOAL.y - robotLoc.y)
      distRobotToGoal = math.sqrt(distRobot_x**2+distRobot_y**2)
      
      distBall_x = (HOME_GOAL.x - robotLoc.x)
      distBall_y = (HOME_GOAL.y - robotLoc.y)
      distBallToGoal = math.sqrt(distBall_x**2+distBall_y**2)
      
      if distRobotToGoal < distBallToGoal:
        return True
      else:
        return False
    
    @staticmethod
    #behind_x is the allowed distance behind the ball the robot can go while still rushing the ball
    def isBallInFrontOfRobot(robotState,ball,param_x = .5, param_y = .04, behind_x = 0):
        refPoint = Point(ball.x-robotState.x,ball.y - robotState.y)
        if refPoint.x < 0:
          if abs(refPoint.x) < behind_x:
            print "behind ball %.2f m but still rushing" %(abs(refPoint.x))
            refPoint.x = -refPoint.x
          else:
            print "ball is too far behind, stopping rush"
            return False
        #print "refence from robot: %f, %f" % (refPoint.x,refPoint.y)
        #print "angle: %f" % robotState.theta
        rotatedPoint = MotionSkills.rotatePointByAngle(refPoint, robotState.theta)
        #print "rotatedPoint to robot frame: %f, %f" % (rotatedPoint.x,rotatedPoint.y)
        angleToGoal = MotionSkills.angleBetweenPoints(robotState, HOME_GOAL)
        angleDiff = abs(angleToGoal-robotState.theta)
        if angleDiff > math.pi:
            angleDiff = (2*math.pi - angleDiff)
        #print 'angle difference: %d' %((angleDiff/math.pi)*180)
        
        if rotatedPoint.x > 0 and rotatedPoint.x < param_x and rotatedPoint.y < param_y and rotatedPoint.y > -param_y:
            
            #print "Behind Ball!"
            if angleDiff < 0.15:
                #print "Angled toward Goal"
                return True
            print "Not angled to goal!"
            return False
        print "Not Behind Ball!"
        return False
        
        

    
