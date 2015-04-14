/*
 * ComputerVision.h
 *
 *  Created on: Mar 14, 2015
 *      Author: ecestudent
 */

#ifndef COMPUTERVISION_H_
#define COMPUTERVISION_H_

#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv/cv.h>
#include <pthread.h>
#include <semaphore.h>
#include <queue>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robot_soccer/locations.h"

typedef struct {
  ros::Time timestamp;
  std::vector<char> image;
} FrameRaw;

typedef struct {
  ros::Time timestamp;
  cv::Mat image;
  cv::Mat HSV;
} FrameMat;

#define PI 3.14159265
#define MIN_CHANGE 0
#define MAX_CHANGE 1000

//default capture width and height
#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480

#define HOME 1
#define AWAY 2

#define GUI 1
#define NO_GUI 2

#define MIN_BUFFER_SIZE 3
#define PRINT_FREQ 30

//----------------------------------------------------------------------------
// Extern variables to share between files
// All of these values are defined in ComputerVision.cpp
//----------------------------------------------------------------------------

// Which TEAM are we on?
extern int TEAM;

// Field variables
extern int field_width;
extern int field_height;
extern int field_center_x;
extern int field_center_y;

//initial min and max HSV filter values.
//these will be changed using trackbars
extern int H_MIN;
extern int H_MAX;
extern int S_MIN;
extern int S_MAX;
extern int V_MIN;
extern int V_MAX;

//min and max field variable values
extern int field_height_min;
extern int field_width_min;
extern int field_center_x_min;
extern int field_center_y_min;
extern int field_height_max;
extern int field_width_max;
extern int field_center_x_max;
extern int field_center_y_max;

//max number of objects to be detected in frame
extern const int MAX_NUM_OBJECTS;

//minimum and maximum object area
extern const int MIN_OBJECT_AREA;
extern const int MAX_OBJECT_AREA;

//names that will appear at the top of each window
extern const std::string windowName;
extern const std::string windowName1;
extern const std::string windowName2;
extern const std::string windowName3;
extern const std::string trackbarWindowName;

// Camera Calibration Data
extern double dist_coeff[5][1];
extern double cam_matrix[3][3];


// Shared Utility Functions
void createHSVTrackbars();
std::string intToString(int number);
cv::Point convertCoordinates(cv::Point imageCoordinates);
void on_trackbar( int, void* );
void morphOps(cv::Mat &thresh);
void undistortImage(cv::Mat &source);

#endif /* COMPUTERVISION_H_ */
