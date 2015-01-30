//============================================================================
// Name : Ball.h
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Outlines the Ball object.
//============================================================================

#ifndef BALL_H_
#define BALL_H_

class Ball {
public:

	// Constructor
	Ball();

	// Destructor
	virtual ~Ball();

	// Setters and Getters
	void set_x_pos(int x);
	int get_x_pos();

	void set_y_pos(int x);
	int get_y_pos();

private:
	int x_pos, y_pos;
};

#endif /* BALL_H_ */
