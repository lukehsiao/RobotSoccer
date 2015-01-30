//============================================================================
// Name : Robot.h
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Outlines the Robot object.
//============================================================================

#ifndef ROBOT_H_
#define ROBOT_H_

#define HOME 1
#define AWAY 2

class Robot {
public:
	Robot();
	virtual ~Robot();


	// Setters and Getters
	void set_x_pos(int x);
	int get_x_pos();

	void set_y_pos(int x);
	int get_y_pos();

	void setTeam(int team);
	int getTeam();

	int getAngle();
	void setAngle(int newAngle);

private:
  int x_pos, y_pos;
  int angle;
  int team;
};

#endif /* ROBOT_H_ */
