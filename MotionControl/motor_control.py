#!/usr/bin/env python
import rospy
from roboclaw import *
import math
from std_msgs.msg import String
from robot_soccer.msg import velocities
from robot_soccer.msg import locations
import mat
import param
#241p/meter
count = 0;
endcount = 0;
callbackf = 10;#times/sec

def radianToQpps(radian):
  result = int(radian * 19820.0 / (2*math.pi))
  #print result
  if result > 308420:
    return 308420
  elif result < -308420:
    return -308420
  else:
    return result
  
#*********************** Skills **************************


# with no good vison method

def skill_go_to_point_N_angle(x_pos,y_pos,data):
#skill - go to a given point
  vx = -(1.0/241.0)*(data.home1_x - x_pos)/2
  vy = -(1.0/241.0)*(data.home1_y - y_pos)/2
  theta_d = angle;
  omega = -param.control_k_phi*(data.home1_theta - theta_d);
  return mat.getWheelVelTheta(vx,vy,omega,data.home1_theta*math.pi/180.0)

#def go_behind_ball_rush_goal(data):
  #if count == 0:
  #  if data.home1_x < data.ball_x:
# with no good vison method
def skill_go_to_point(data,x_pos,y_pos):
  if(skill_go_to_point.counter == 0):
  #need to change speed
    theta_btw_ball_N_goal = math.atan2(param.goal_y - data.ball_y, param.goal_x - data.ball_x)
    print "ball_x is"
    print data.ball_x
    print "ball_y is"
    print data.ball_y
    behind_ball_x = data.ball_x - 5*math.cos(theta_btw_ball_N_goal)
    behind_ball_y = data.ball_y - 5*math.sin(theta_btw_ball_N_goal) #in pixel
    print "behind ball x is"
    print behind_ball_x
    print "behind ball y is"
    print behind_ball_y
    distance = sqrt((behind_ball_y -data.home1_y )**2 + (behind_ball_x -data.home1_x)**2)
    distance = distance * (1.0/241.0)#convert from pix to m
    v = 0.005 # m/s
    t = distance/v
    endcount = t*callbackf

    vx = v*(data.home1_x - x_pos)/distance
    vy = v*(data.home1_y - data.ball_y)/distance
    vx = 0
    vy = 0
    return mat.getWheelVelTheta(vx, vy, desired_w_world, data.home1_theta*math.pi/180.0)
  if(skill_go_to_point.counter > endcount):
    return mat.getWheelVelTheta(0, 0, 0, data.home1_theta*math.pi/180.0)

skill_go_to_point.counter = 0

# with no good vison method
def skill_turn_towards_behind_ball(data,x_pos,y_pos):

  return mat.getWheelVelTheta(0, 0,  1, data.home1_theta*math.pi/180.0)
# skill - turn towards a point
  #x_pos = data.ball_x;
  #y_pos = data.ball_y;
  endcount = 0;
  testendcount = 5;
  desired_w_world = radianToQpps(0.0006)#turn by 30 degree
  if skill_turn_towards_behind_ball.counter == 0:
    theta_btw_ball_N_goal = math.atan2(param.goal_y - data.ball_y, param.goal_x - data.ball_x)
    behind_ball_x = data.ball_x - 5*math.cos(theta_btw_ball_N_goal)
    behind_ball_y = data.ball_y - 5*math.sin(theta_btw_ball_N_goal) #in pixel
    change_theta = (behind_ball_y - data.home1_y)/(behind_ball_x - data.home1_x) - (data.home1_theta + math.pi/2)
    t = math.fabs(change_theta)/0.0006
    print "change_theta is "
    print change_theta
    endcount = t*callbackf
    #0.01 is one circle called once
   
    print "counter is 0"
    print skill_turn_towards_behind_ball.counter
    skill_turn_towards_behind_ball.counter+= 1
    print "counter is 1 "
    print skill_turn_towards_behind_ball.counter
    return mat.getWheelVelTheta(0, 0, 1, data.home1_theta*math.pi/180.0)
  elif skill_turn_towards_behind_ball.counter < testendcount:
    print "this should be printed 5 times endcount is "
    print skill_turn_towards_behind_ball.counter
    skill_turn_towards_behind_ball.counter+=1
    return mat.getWheelVelTheta(0, 0,  1, data.home1_theta*math.pi/180.0)
    
  else:
    print "counter is "
    print skill_turn_towards_behind_ball.counter
    return mat.getWheelVelTheta(0, 0, 0, data.home1_theta*math.pi/180.0)

skill_turn_towards_behind_ball.counter = 0

# with good vison method
def skill_go_to_behind_ball(data):
  print "ball_x is"
  print data.ball_x
  print "ball_y is"
  print data.ball_y
  
  if(skill_go_to_behind_ball.done == 0):
    """
    theta_btw_ball_N_goal = math.atan2(param.goal_y - data.ball_y, param.goal_x - data.ball_x)
    behind_ball_x = data.ball_x - 5*math.cos(theta_btw_ball_N_goal)
    behind_ball_y = data.ball_y - 5*math.sin(theta_btw_ball_N_goal) #in pixel
    print "behind ball x is"
    print behind_ball_x
    print "behind ball y is"
    print behind_ball_y
    """
    if(data.ball_y > 20):
      behind_ball_y = data.ball_y +15
    elif(data.ball_y < -20):
      behind_ball_y = data.ball_y -15
    else:
      behind_ball_y = data.ball_y

    behind_ball_x = data.ball_x - 5
    vx = -param.control_k_vx*(1.0/241.0)*(data.home1_x - behind_ball_x)/5
    vy = -param.control_k_vy*(1.0/241.0)*(data.home1_y - behind_ball_y)/5
    print "vx is"
    print vx
    print "vy is"
    print vy
    if (vx < 0.05 and vy < 0.05):
      skill_go_to_behind_ball.done = 1
    v1,v2,v3 = mat.getWheelVelTheta(vx, vy, 0, data.home1_theta*math.pi/180.0)
  return (v1,v2,v3,skill_go_to_behind_ball.done)
skill_go_to_behind_ball.done = 0

# with good vison method

def skill_rush_goal_when_behind_ball(data):
  print "rush goal"
  vx = -param.control_k_vx*(1.0/241.0)*(data.home1_x - param.goal_x)
  vy = -param.control_k_vy*(1.0/241.0)*(data.home1_y - param.goal_y)
  return mat.getWheelVelTheta(vx, vy, 0, data.home1_theta*math.pi/180.0)



# with good vison method
def skill_go_to_ball_then_rush_goal(data):
  v1,v2,v3 = (0,0,0)
  if(skill_go_to_ball_then_rush_goal.state1done == 0):
    v1, v2, v3, skill_go_to_ball_then_rush_goal.state1done = skill_go_to_behind_ball(data);
    
    return v1,v2,v3
  else:
    v1,v2,v3 = skill_rush_goal_when_behind_ball(data)
    return v1,v2,v3
  return v1,v2,v3
skill_go_to_ball_then_rush_goal.state1done = 0


def skill_ny(data):
    print "ny"
    vx = 0;
    vy = -param.control_k_vy*(1.0/241.0)*(30)
    print "vx is"
    print vx
    print "vy is"
    print vy
    return mat.getWheelVelTheta(vx, vy, 0, data.home1_theta*math.pi/180.0)

def skill_nx(data):
    print "nx"
    vy = 0;
    vx = -param.control_k_vx*(1.0/241.0)*(30)
    print "vx is"
    print vx
    print "vy is"
    print vy
    return mat.getWheelVelTheta(vx, vy,0, data.home1_theta*math.pi/180.0)

def skill_py(data):
    print "py"
    vx = 0;
    vy = -param.control_k_vy*(1.0/241.0)*(-30)
    print "vx is"
    print vx
    print "vy is"
    print vy
    return mat.getWheelVelTheta(vx, vy, 0, data.home1_theta*math.pi/180.0)

def skill_px(data):
    print "px"
    vy = 0;
    vx = -param.control_k_vx*(1.0/241.0)*(-30)
    print "vx is"
    print vx
    print "vy is"
    print vy
    return mat.getWheelVelTheta(vx, vy,0, data.home1_theta*math.pi/180.0)

# with good vison method
def skill_follow_ball_on_line(data,x_pos):
    print "ball_x is"
    print data.ball_x
    print "ball_y is"
    print data.ball_y
# skill - follow ball on line
#   follows the y-position of the ball, while maintaining x position at
#   x_pos.  Angle always faces the goal.
    target_x = x_pos;
    target_y = data.ball_y
    if(data.ball_x <= 270):
	  diffx = data.home1_x - target_x
	  diffy = data.home1_y - target_y
          print "robot x is"
          print data.home1_x
          print "robot y is"
          print data.home1_y
	  vx = -param.control_k_vx*(1.0/241.0)*(data.home1_x - target_x)
	  vy = -param.control_k_vy*(1.0/241.0)*(data.home1_y - target_y)
          
        

	  #theta_d = math.atan2(param.goal_y-data.home1_y, param.goal_x-data.home1_x)
	  #omega = -param.control_k_phi*(data.home1_theta - theta_d)
	 

	  """
	  
	  if(diffx <= 1):
	    skill_follow_ball_on_line.done_x_count = 1
	    vx = 0
	  if(diffy <= 1):
	    skill_follow_ball_on_line.done_y_count = 1
	    vy = 0
	  """

	  return mat.getWheelVelTheta(0,vy,0,data.home1_theta*math.pi/180.0)
    else:
          return mat.getWheelVelTheta(0,0,0,data.home1_theta*math.pi/180.0)
skill_follow_ball_on_line.done_x_count = 0
skill_follow_ball_on_line.done_y_count = 0


      


def callback(data):
   """
  v = sqrt(vx*vx+vy*vy)
          print "v is"
          print v;
          print "vx is"
          print vx;
          print "vy is"
          print vy;
          vxx = vx * param.v_max/v
          vyy = vy * param.v_max/v
          print "vxx is"
          print vxx;
          print "vyy is"
          print vyy;
  ############       test box    #######################
  if(callback.counter < 20):
    v1,v2,v3 = skill_ny(data)
  elif(callback.counter > 20 and callback.counter < 40 ):
    v1,v2,v3 = skill_nx(data)
  elif(callback.counter > 40 and callback.counter < 60 ):
    v1,v2,v3 = skill_py(data)
  elif(callback.counter > 60 and callback.counter < 80 ):
    v1,v2,v3 = skill_px(data)
  else:
    v1,v2,v3 = (0,0,0)
  ####################follow_ball_on_line#########################
  """

   x_pos = -253
   #v1,v2,v3 = skill_follow_ball_on_line(data,x_pos)
####################       score      ####################
   v1,v2,v3 = skill_go_to_ball_then_rush_goal(data)

#  v1,v2,v3 = mat.getWheelVelTheta(0,0,0,data.home1_theta*math.pi/180.0)
   if(callback.counter%15 == 0):
    v1,v2,v3 = (0,0,0)
   s1 = radianToQpps(v1)
   s2 = radianToQpps(v2)
   s3 = radianToQpps(v3)
   SetM1SpeedAccel(128,abs(s1),s1)
   SetM2SpeedAccel(128,abs(s2),s2)
   SetM1SpeedAccel(129,abs(s3),s3)
   callback.counter += 1
"""
#    rospy.loginfo("wheel1 #d", data.wheel1)
#    rospy.loginfo("wheel2 #d", data.wheel2)
#    rospy.loginfo("wheel3 #d", data.wheel3)
    
    #*****************************************************
    
    # This is where all of your control code will go

    #*****************************************************


  v1,v2,v3 = skill_follow_ball_on_line(data,0)
  
  x = -float(data.home1_x)*(1.397/400.0)
  y = -float(data.home1_y)*(1.397/400.0)
  theta = float(data.home1_theta-270)*(math.pi/180.0)
  "This is a new print"
  print x
  print y
  print theta
  v1,v2,v3 = mat.getWheelVelTheta(x,y,0.0,theta)
 # mat.getWheelVelTheta(vx,vy,omega,data.home1_theta);
 """
  
callback.counter = 0


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node('motor_control', anonymous=True)

    # This subscribes to the velTopic topic expecting the 'velocities' message
    rospy.Subscriber("locTopic", locations, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()