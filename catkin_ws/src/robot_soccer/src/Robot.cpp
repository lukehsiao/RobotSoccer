//============================================================================
// Name : Robot.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Implementation of Robot methods
//============================================================================

#include "Robot.h"

using namespace cv;

Robot::Robot(int TEAM) : Object() {
  Robot::team = TEAM;
  Robot::angle = 0;
  Robot::old_angle = 0;
  // Hard code team colors here
  if (team == HOME) {
    setHSVmin(cv::Scalar(0,0,235));
    setHSVmax(cv::Scalar(5,255,255));
  }
  else if (team == AWAY) {
    setHSVmin(cv::Scalar(103,67,0));
    setHSVmax(cv::Scalar(138,217,255));
  }
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}

// Setters and Getters
void Robot::setAngle(int newAngle) {
  Robot::old_angle = Robot::angle;
  Robot::angle = newAngle;
}

int Robot::getAngle() {
  return Robot::angle;
}

int Robot::getOldAngle() {
  return Robot::old_angle;
}

// Set which team the robot is on. 1 = HOME, 2 = AWAY
void Robot::setTeam(int team) {
  Robot::team = team;
}

// Returns which team the robot is on. 1 = HOME, 2 = AWAY
int Robot::getTeam() {
  return Robot::team;
}
