//============================================================================
// Name : Ball.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Implementation of Ball methods
//============================================================================

#include "Ball.h"

Ball::Ball() : Object() {
  setHSVmin(cv::Scalar(1,148,172));
  setHSVmax(cv::Scalar(70,255,238));
}

Ball::~Ball() {
	// TODO Auto-generated destructor stub
}


