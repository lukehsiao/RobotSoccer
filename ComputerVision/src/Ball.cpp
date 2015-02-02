//============================================================================
// Name : Ball.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Implementation of Ball methods
//============================================================================

#include "Ball.h"

Ball::Ball() : Object() {
  setHSVmin(cv::Scalar(20,20,100));
  setHSVmax(cv::Scalar(33,243,238));
}

Ball::~Ball() {
	// TODO Auto-generated destructor stub
}


