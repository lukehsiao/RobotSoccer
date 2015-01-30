//============================================================================
// Name : Ball.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Implementation of Ball methods
//============================================================================

#include "Ball.h"

Ball::Ball() : Object() {
  setHSVmin(cv::Scalar(22,70,0));
  setHSVmax(cv::Scalar(63,255,255));
}

Ball::~Ball() {
	// TODO Auto-generated destructor stub
}


