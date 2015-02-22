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

#define HOME 1
#define AWAY 2

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

  private:
    int angle;
    int old_angle;
    int team;
};

#endif /* ROBOT_H_ */
