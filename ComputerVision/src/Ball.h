//============================================================================
// Name : Ball.h
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Outlines the Ball object.
//============================================================================

#ifndef BALL_H_
#define BALL_H_

#include "Object.h"

class Ball: public Object {
  public:

    // Constructor
    Ball();

    // Destructor
    virtual ~Ball();

    // Ball Methods
    void calibrateBall(cv::VideoCapture capture);
    void trackFilteredBall(cv::Mat threshold, cv::Mat HSV, cv::Mat &cameraFeed);
    void drawBall(cv::Mat &frame);
};

#endif /* BALL_H_ */
