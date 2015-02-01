//============================================================================
// Name : Ball.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Implementation of Ball methods
//============================================================================

#include "Ball.h"

Ball::Ball() : Object() {
  setHSVmin(cv::Scalar(20,148,69));
  setHSVmax(cv::Scalar(79,243,238));
}

Ball::~Ball() {
	// TODO Auto-generated destructor stub
}


