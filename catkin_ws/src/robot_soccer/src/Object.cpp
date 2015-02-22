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
  Object::img_x = 455;
  Object::img_y = 240;
  Object::old_x_pos = 0;
  Object::old_y_pos = 0;
  Object::HSVmax = 0;
  Object::HSVmin = 0;
}

Object::~Object() {
  // TODO Auto-generated destructor stub
}

// Setters and Getters
void Object::set_x_pos(int x) {
  Object::old_x_pos = Object::x_pos; // save old position
  Object::x_pos = x;
}

int Object::get_x_pos() {
  return Object::x_pos;
}

void Object::set_img_x(int x) {
  Object::img_x = x;
}

int Object::get_img_x() {
  return Object::img_x;
}

void Object::set_img_y(int y) {
  Object::img_y = y;
}

int Object::get_img_y() {
  return Object::img_y;
}

void Object::set_y_pos(int y) {
  Object::old_y_pos = Object::y_pos; // save old position
  Object::y_pos = y;
}

int Object::get_y_pos() {
  return Object::y_pos;
}

int Object::get_old_y() {
  return Object::old_y_pos;
}

int Object::get_old_x() {
  return Object::old_x_pos;
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
