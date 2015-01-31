//============================================================================
// Name : Ball.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Implementation of Ball methods
//============================================================================

#include "Ball.h"

Ball::Ball() : Object() {
  setHSVmin(cv::Scalar(0,162,44));
  setHSVmax(cv::Scalar(20,255,255));
}

Ball::~Ball() {
	// TODO Auto-generated destructor stub
}


