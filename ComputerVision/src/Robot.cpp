//============================================================================
// Name : Robot.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Implementation of Robot methods
//============================================================================

#include "Robot.h"

using namespace cv;

Robot::Robot() : Object() {
  Robot::team = 0;
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
