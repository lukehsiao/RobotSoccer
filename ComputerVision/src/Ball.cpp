//============================================================================
// Name : Ball.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Implementation of Ball methods
//============================================================================

#include "Ball.h"

Ball::Ball() : Object() {
  setHSVmin(cv::Scalar(22,120,0));
  setHSVmax(cv::Scalar(38,255,255));
}

Ball::~Ball() {
	// TODO Auto-generated destructor stub
}


