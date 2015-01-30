/*
 * Object.cpp
 *
 *  Created on: Jan 30, 2015
 *      Author: lukehsiao
 */

#include "Object.h"

using namespace cv;

Object::Object() {
  Object::x_pos = 0;
  Object::y_pos = 0;
  Object::HSVmax = 0;
  Object::HSVmin = 0;
}

Object::~Object() {
  // TODO Auto-generated destructor stub
}

// Setters and Getters
void Object::set_x_pos(int x) {
  Object::x_pos = x;
}

int Object::get_x_pos() {
  return Object::x_pos;
}

void Object::set_y_pos(int y) {
  Object::y_pos = y;
}

int Object::get_y_pos() {
  return Object::y_pos;
}

cv::Scalar Object::getHSVmin() {
  return Object::HSVmin;
}

void Object::setHSVmin(cv::Scalar min) {
  Object::HSVmin = min;
}

cv::Scalar Object::getHSVmax() {
  return Object::HSVmax;
}

void Object::setHSVmax(cv::Scalar max) {
  Object::HSVmax = max;
}
