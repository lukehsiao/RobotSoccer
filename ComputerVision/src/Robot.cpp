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
  // Hard code team colors here
  if (team == HOME) {
    setHSVmin(cv::Scalar(0,0,171));
    setHSVmax(cv::Scalar(10,255,255));
  }
  else if (team == AWAY) {
    setHSVmin(cv::Scalar(0,0,0));
    setHSVmax(cv::Scalar(255,255,255));
  }
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}

// Setters and Getters
void Robot::setAngle(int newAngle) {
  Robot::angle = newAngle;
}

int Robot::getAngle() {
  return Robot::angle;
}

// Set which team the robot is on. 1 = HOME, 2 = AWAY
void Robot::setTeam(int team) {
  Robot::team = team;
}

// Returns which team the robot is on. 1 = HOME, 2 = AWAY
int Robot::getTeam() {
  return Robot::team;
}
