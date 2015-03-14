//============================================================================
// Name : Robot.h
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Outlines the Robot object.
//============================================================================

#ifndef ROBOT_H_
#define ROBOT_H_

#include "Object.h"

class Robot: public Object {
  public:
    Robot(int TEAM);
    virtual ~Robot();

    // Setters and Getters
    void setTeam(int team);
    int getTeam();

    int getOldAngle();

    int getAngle();
    void setAngle(int newAngle);

    // Robot methods
    void calibrateRobot(cv::VideoCapture capture);
    void trackFilteredRobot(cv::Mat threshold, cv::Mat HSV, cv::Mat &cameraFeed);
    void drawRobot(cv::Mat &frame);

  private:
    int angle;
    int old_angle;
    int team;
};

#endif /* ROBOT_H_ */
