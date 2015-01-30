//============================================================================
// Name : Ball.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Implementation of Ball methods
//============================================================================

#include "Ball.h"

Ball::Ball() {
  Ball::x_pos = 0;
  Ball::y_pos = 0;
}

Ball::~Ball() {
	// TODO Auto-generated destructor stub
}

// Setters and Getters
void Ball::set_x_pos(int x) {
  Ball::x_pos = x;
}

int Ball::get_x_pos() {
  return Ball::x_pos;
}

void Ball::set_y_pos(int x) {
  Ball::y_pos = x;
}

int Ball::get_y_pos() {
  return Ball::y_pos;
}
