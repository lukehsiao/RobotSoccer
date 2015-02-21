//============================================================================
// Name : ComputerVision.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : This program receives video data from the Soccer Field's
// overhead camera, processes the images, and outputs
// the (x,y) positions of all 4 robots, the ball, and all
// robot's orientations.
//============================================================================

#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include "Ball.h"
#include "Robot.h"
#include "Object.h"

#define PI 3.14159265
#define MIN_CHANGE 5
#define MAX_CHANGE 1000

// Constants for determining field coordinate systems
//#define FIELD_WIDTH 640// 790
//#define FIELD_HEIGHT 400
//#define FIELD_CENTER_X 320//455
//#define FIELD_CENTER_Y 200//240

using namespace cv;

// Field variables
int field_width;
int field_height;
int field_center_x;
int field_center_y;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

//min and max field variable values
int field_height_min = 0;
int field_width_min = 0;
int field_center_x_min = 0;
int field_center_y_min = 0;
int field_height_max = 600;
int field_width_max = 900;
int field_center_x_max = 500;
int field_center_y_max = 500;

//default capture width and height
const int FRAME_WIDTH = 1024;
const int FRAME_HEIGHT = 768;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;

//minimum and maximum object area
const int MIN_OBJECT_AREA = 7*7;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

// This function is called whenever a trackbar changes
void on_trackbar( int, void* ) {
  // Does nothing
}

string intToString(int number) {
  std::stringstream ss;
  ss << number;
  return ss.str();
}


void createHSVTrackbars() {
	//create window for trackbars
	namedWindow(trackbarWindowName,0);

	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
	createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
	createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
	createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
	createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
	createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
}

//Converts from image coordinates to field coordinates
Point convertCoordinates(Point imageCoordinates) {
  int img_x = imageCoordinates.x;
  int img_y = imageCoordinates.y;

  int field_x;
  int field_y;
  Point result;

  field_x = img_x - field_center_x;

  field_y = field_center_y - img_y;

  result.x = field_x;
  result.y = field_y;
  return result;
}

// Places a small circle on the object
void drawObject(int x,int y,Mat &frame) {
	circle(frame,cv::Point(x,y),10,cv::Scalar(0,0,255));
	putText(frame,intToString(x)+ " , " + intToString(y),
	        Point(x,y+20),1,1,Scalar(0,255,0));
}

void drawBall(Ball soccerBall, Mat &frame) {
  int x = soccerBall.get_img_x();
  int y = soccerBall.get_img_y();

  int real_x = soccerBall.get_x_pos();
  int real_y = soccerBall.get_y_pos();
  circle(frame,cv::Point(x,y),10,cv::Scalar(0,0,255));
  putText(frame,"(" + intToString(real_x)+ "," + intToString(real_y) + ")",
          Point(x,y+20),1,1,Scalar(0,255,0));
  putText(frame, "Ball", Point(x+25,y+35),1,1,Scalar(0,255,0));
}

void drawRobot(Robot newRobot, Mat &frame) {
  int x = newRobot.get_img_x();
  int y = newRobot.get_img_y();
  int team = newRobot.getTeam();
  int angle = newRobot.getAngle();

  int real_x = newRobot.get_x_pos();
  int real_y = newRobot.get_y_pos();

  // TODO there may be some error in this value to do compensating for the noise
  circle(frame,cv::Point(x,y),10, Scalar(0,0,255));
  putText(frame,"(" + intToString(real_x)+ "," + intToString(real_y) + ")",
          Point(x,y+20),1,1,Scalar(0,255,0));
  putText(frame, "Robot", Point(x+17,y+35),1,1,Scalar(0,255,0));
  putText(frame, "Team " + intToString(team), Point(x+17,y+50),1,1,Scalar(0,255,0));
  putText(frame, "Angle: " + intToString(angle), Point(x+17,y+65),1,1,Scalar(0,255,0));
}

// Draws all robot objects that are found
void drawAllRobots(vector<Robot> robots_to_draw, Mat &frame) {
  // Iterate through all found robots and draw them
  for (unsigned i = 0; i < robots_to_draw.size(); i++) {
    drawRobot(robots_to_draw.at(i), frame);
  }
}

// This function reduces the noise of the image by eroding the image first
// then dialating the remaining image to produce cleaner objects
void morphOps(Mat &thresh) {
	//create structuring element that will be used to "dilate" and "erode" image.

	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(2,2));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(7,7));

	erode(thresh,thresh,erodeElement);
  dilate(thresh,thresh,dilateElement);

	erode(thresh,thresh,erodeElement);
	dilate(thresh,thresh,dilateElement);
}

// Function specific for tracking robots. Will calculate the center of the robot as
// well as the it's angle in relation to the horizontal.
void trackFilteredRobot(Robot &robot, Mat threshold, Mat HSV, Mat &cameraFeed) {
  Mat temp;
  threshold.copyTo(temp);

  int c1 = 0, c2=1;   // c1 = Center Point of Big Circle and c2 = Center Point of Small Circle

  //these two vectors needed for output of findContours
  vector< vector<Point> > contours;
  vector<Vec4i> hierarchy;

  //find contours of filtered image using openCV findContours function
  findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );

  //use moments method to find our filtered object
  //TODO(lukehsiao) This WILL break if there are more than 2 objects found. Segmentation has to be really good.
  if (contours.size() == 2) {

    // Identify the bigger object
    if (contourArea(contours[0]) < contourArea(contours[1])) {
      c1 = 1;
      c2 = 0;
    }
    else {
      c1 = 0;
      c2 = 1;
    }

    // Get moments
    vector<Moments> robotMoments(contours.size());
    for (unsigned i = 0; i < contours.size(); i++) {
      robotMoments[i] = moments(contours[i], false);
    }

    // Find centers
    vector<Point2f> centerPoints(contours.size());
    for (unsigned i = 0; i < contours.size(); i++) {
      centerPoints[i] = Point2f(robotMoments[i].m10/robotMoments[i].m00,
                                robotMoments[i].m01/robotMoments[i].m00);
    }

    // Mark the bigger circle
    circle(cameraFeed, centerPoints[c1], 9, Scalar(255,0,0), -1, 8, 0);

    //Draw line between centers
    line(cameraFeed, centerPoints[c1], centerPoints[c2], Scalar(0,0,255), 4, 8, 0);

    //Calculate the angle
    float angle = (atan2(centerPoints[c2].y - centerPoints[c1].y,
                         centerPoints[c2].x - centerPoints[c1].x))*(180/PI);

    //Convert to int
    int intAngle = (int) angle;
    if (intAngle <= 0) {
      intAngle = intAngle * (-1); // make positive again
    }
    else {
      intAngle = 360-intAngle;
    }

    // TODO Filtering bad data (if the change in xpos or ypos is too large, ignore data
    // Set Robot variables

    if (abs(intAngle - robot.getOldAngle()) > 5) {
      robot.setAngle(intAngle);
    }

    Point fieldPosition = convertCoordinates(Point((int)centerPoints[c1].x,
                                                   (int)centerPoints[c1].y));
    if (abs(fieldPosition.x - robot.get_old_x()) > MIN_CHANGE &&
        abs(fieldPosition.x - robot.get_old_x()) < MAX_CHANGE) {
      robot.set_x_pos(fieldPosition.x);
      robot.set_img_x((int)centerPoints[c1].x);
    }
    if (abs(fieldPosition.y - robot.get_old_y()) > MIN_CHANGE &&
        abs(fieldPosition.y - robot.get_old_y()) < MAX_CHANGE) {
      robot.set_y_pos(fieldPosition.y);
      robot.set_img_y((int)centerPoints[c1].y);
    }

    drawRobot(robot, cameraFeed);
  }
}

// Finds the contours (outlines) of the now filtered image and determine's its
// center by examining its moments.

void trackFilteredBall(Ball &ball, Mat threshold, Mat HSV, Mat &cameraFeed) {

	Mat temp;
	threshold.copyTo(temp);

	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;

	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );

	//use moments method to find our filtered object
	//double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA) {
          Point fieldPosition = convertCoordinates(Point(moment.m10/area,
                                                         moment.m01/area));
          if(abs(fieldPosition.x - ball.get_old_x()) > MIN_CHANGE &&
             abs(fieldPosition.x - ball.get_old_x()) < MAX_CHANGE) {
            ball.set_x_pos(fieldPosition.x);
            ball.set_img_x(moment.m10/area);
          }
          if(abs(fieldPosition.y - ball.get_old_y()) > MIN_CHANGE &&
             abs(fieldPosition.y - ball.get_old_y()) < MAX_CHANGE) {
            ball.set_y_pos(fieldPosition.y);
            ball.set_img_y(moment.m01/area);
          }
					objectFound = true;
				}
				else {
				  objectFound = false;
				}
			}
			//let user know you found an object
			if(objectFound ==true){
				//draw object location on screen
				drawBall(ball,cameraFeed);}

		}
		else {
		  putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
		}
	}
}

// Used for tracking or calibrating to a generic colored object
void trackFilteredObject(Mat threshold, Mat HSV, Mat &cameraFeed) {
  int x, y;
  Mat temp;
  threshold.copyTo(temp);

  //these two vectors needed for output of findContours
  vector< vector<Point> > contours;
  vector<Vec4i> hierarchy;

  //find contours of filtered image using openCV findContours function
  findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

  //use moments method to find our filtered object
  //double refArea = 0;
  bool objectFound = false;
  if (hierarchy.size() > 0) {
    int numObjects = hierarchy.size();
    //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
    if(numObjects<MAX_NUM_OBJECTS){
      for (int index = 0; index >= 0; index = hierarchy[index][0]) {

        Moments moment = moments((Mat)contours[index]);
        double area = moment.m00;

        //if the area is less than 20 px by 20px then it is probably just noise
        //if the area is the same as the 3/2 of the image size, probably just a bad filter
        //we only want the object with the largest area so we safe a reference area each
        //iteration and compare it to the area in the next iteration.
        if(area>MIN_OBJECT_AREA){
          x = moment.m10/area;
          y = moment.m01/area;
          objectFound = true;
        }
        else {
          objectFound = false;
        }
      }
      //let user know you found an object
      if(objectFound ==true){
        //draw object location on screen
        drawObject(x, y,cameraFeed);}

    }
    else {
      putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
    }
  }
}

// Generate prompts to calibrate colors for the Home1 robots
void calibrateRobot_Home1(VideoCapture capture, Robot &Home1) {
  Mat cameraFeed;
  Mat HSV;
  Mat threshold;
  int h_min;
  int h_max;
  int s_min;
  int s_max;
  int v_min;
  int v_max;
  int field_origin_x;
  int field_origin_y;

  //create trackbars
  createHSVTrackbars();

  // Set Trackbar intial values to near Yellow
  setTrackbarPos( "H_MIN", trackbarWindowName, 0);
  setTrackbarPos( "H_MAX", trackbarWindowName, 35);
  setTrackbarPos( "S_MIN", trackbarWindowName, 0);
  setTrackbarPos( "S_MAX", trackbarWindowName, 255);
  setTrackbarPos( "V_MIN", trackbarWindowName, 225);
  setTrackbarPos( "V_MAX", trackbarWindowName, 255);

  // Wait forever until user sets the values
   while (1) {
      //store image to matrix
      capture.read(cameraFeed);

      //convert frame from BGR to HSV colorspace
      field_origin_x = field_center_x - (field_width/2);
      field_origin_y = field_center_y - (field_height/2);
      Rect myROI(field_origin_x,field_origin_y,field_width, field_height);
      cameraFeed = cameraFeed(myROI);
      cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);

      //if in calibration mode, we track objects based on the HSV slider values.
      inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);

      // Erode, then dialate to get a cleaner image
      morphOps(threshold);

      h_min = getTrackbarPos( "H_MIN", trackbarWindowName);
      h_max = getTrackbarPos( "H_MAX", trackbarWindowName);
      s_min = getTrackbarPos( "S_MIN", trackbarWindowName);
      s_max = getTrackbarPos( "S_MAX", trackbarWindowName);
      v_min = getTrackbarPos( "V_MIN", trackbarWindowName);
      v_max = getTrackbarPos( "V_MAX", trackbarWindowName);

      Scalar hsv_min(h_min, s_min, v_min);
      Scalar hsv_max(h_max, s_max, v_max);

      Home1.setHSVmin(hsv_min);
      Home1.setHSVmax(hsv_max);

      trackFilteredRobot(Home1,threshold,HSV,cameraFeed);

      imshow(windowName2,threshold);
      imshow(windowName,cameraFeed);

      char pressedKey;
      pressedKey = cvWaitKey(50); // Wait for user to press 'Enter'
      if (pressedKey == '\n') {

          Scalar hsv_min(h_min, s_min, v_min);
          Scalar hsv_max(h_max, s_max, v_max);

          Home1.setHSVmin(hsv_min);
          Home1.setHSVmax(hsv_max);

          printf("\n\n Robot Home1 HSV Values Saved!\n");
          printf("h_min: %d\n", h_min);
          printf("h_max: %d\n", h_max);
          printf("s_min: %d\n", s_min);
          printf("s_max: %d\n", s_max);
          printf("v_min: %d\n", v_min);
          printf("v_max: %d\n", v_max);

          destroyAllWindows();

          // Reset Globals
          H_MIN = 0;
          H_MAX = 256;
          S_MIN = 0;
          S_MAX = 256;
          V_MIN = 0;
          V_MAX = 256;

          return;
      }
   }


}

// Generates prompts for calibration of the color ball
void calibrateBall(VideoCapture capture, Ball &ball) {
Mat cameraFeed;
Mat HSV;
Mat threshold;
int h_min;
int h_max;
int s_min;
int s_max;
int v_min;
int v_max;
int field_origin_x;
int field_origin_y;

//create trackbars
createHSVTrackbars();

// Set Trackbar intial values to near Yellow
setTrackbarPos( "H_MIN", trackbarWindowName, 20);
setTrackbarPos( "H_MAX", trackbarWindowName, 45);
setTrackbarPos( "S_MIN", trackbarWindowName, 120);
setTrackbarPos( "S_MAX", trackbarWindowName, 255);
setTrackbarPos( "V_MIN", trackbarWindowName, 150);
setTrackbarPos( "V_MAX", trackbarWindowName, 255);

// Wait forever until user sets the values
 while (1) {
    //store image to matrix
    capture.read(cameraFeed);

    //convert frame from BGR to HSV colorspace
    field_origin_x = field_center_x - (field_width/2);
    field_origin_y = field_center_y - (field_height/2);
    Rect myROI(field_origin_x,field_origin_y,field_width, field_height);
    cameraFeed = cameraFeed(myROI);
    cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);

    //if in calibration mode, we track objects based on the HSV slider values.
    inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);

    // Erode, then dialate to get a cleaner image
    morphOps(threshold);



    h_min = getTrackbarPos( "H_MIN", trackbarWindowName);
    h_max = getTrackbarPos( "H_MAX", trackbarWindowName);
    s_min = getTrackbarPos( "S_MIN", trackbarWindowName);
    s_max = getTrackbarPos( "S_MAX", trackbarWindowName);
    v_min = getTrackbarPos( "V_MIN", trackbarWindowName);
    v_max = getTrackbarPos( "V_MAX", trackbarWindowName);

    Scalar hsv_min(h_min, s_min, v_min);
    Scalar hsv_max(h_max, s_max, v_max);

    ball.setHSVmin(hsv_min);
    ball.setHSVmax(hsv_max);

    trackFilteredBall(ball,threshold,HSV,cameraFeed);

    imshow(windowName,cameraFeed);
    imshow(windowName2,threshold);

    char pressedKey;
    pressedKey = cvWaitKey(50); // Wait for user to press 'Enter'
    if (pressedKey == '\n') {
       Scalar hsv_min(h_min, s_min, v_min);
       Scalar hsv_max(h_max, s_max, v_max);

       ball.setHSVmin(hsv_min);
       ball.setHSVmax(hsv_max);

       printf("\n\nBall HSV Values Saved!\n");
       printf("h_min: %d\n", h_min);
       printf("h_max: %d\n", h_max);
       printf("s_min: %d\n", s_min);
       printf("s_max: %d\n", s_max);
       printf("v_min: %d\n", v_min);
       printf("v_max: %d\n", v_max);

       destroyAllWindows();

       // Reset Globals
       H_MIN = 0;
       H_MAX = 256;
       S_MIN = 0;
       S_MAX = 256;
       V_MIN = 0;
       V_MAX = 256;

       return;
     }
   }
}

// Generates prompts for field calibration of size/center
void calibrateField(VideoCapture capture) {
  Mat cameraFeed;
  int field_origin_x;
  int field_origin_y;
  //create window for trackbars
  namedWindow(trackbarWindowName,0);

  //create trackbars and insert them into window
  //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
  //the max value the trackbar can move (eg. H_HIGH),
  //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
  createTrackbar( "Field Center Y", trackbarWindowName, &field_center_y_min, field_center_y_max, on_trackbar );
  createTrackbar( "Field Center X", trackbarWindowName, &field_center_x_min, field_center_x_max, on_trackbar );
  createTrackbar( "Field Height", trackbarWindowName, &field_height_min, field_height_max, on_trackbar );
  createTrackbar( "Field Width", trackbarWindowName, &field_width_min, field_width_max, on_trackbar );

  // Set Trackbar Initial Positions
  setTrackbarPos( "Field Center Y", trackbarWindowName, field_center_y);
  setTrackbarPos( "Field Center X", trackbarWindowName, field_center_x);
  setTrackbarPos( "Field Height", trackbarWindowName, field_height);
  setTrackbarPos( "Field Width", trackbarWindowName, field_width);

  // Wait forever until user sets the values
  while (1) {
    capture.read(cameraFeed);
    // Wait for user to set values
    field_center_y = getTrackbarPos( "Field Center Y", trackbarWindowName);
    field_center_x = getTrackbarPos( "Field Center X", trackbarWindowName);
    field_height = getTrackbarPos( "Field Height", trackbarWindowName);
    field_width = getTrackbarPos( "Field Width", trackbarWindowName);

    field_origin_x = field_center_x - (field_width/2);
    field_origin_y = field_center_y - (field_height/2);

    Rect fieldOutline(field_origin_x, field_origin_y, field_width, field_height);
    rectangle(cameraFeed,fieldOutline,Scalar(255,255,255), 1, 8 ,0);
    imshow(windowName,cameraFeed);

    char pressedKey;
    pressedKey = cvWaitKey(50); // Wait for user to press 'Enter'
    if (pressedKey == '\n') {
      field_center_y = getTrackbarPos( "Field Center Y", trackbarWindowName);
      field_center_x = getTrackbarPos( "Field Center X", trackbarWindowName);
      field_height = getTrackbarPos( "Field Height", trackbarWindowName);
      field_width = getTrackbarPos( "Field Width", trackbarWindowName);

      printf("\n\nField Values Saved!\n");
      printf("Field Center Y: %d\n", field_center_y);
      printf("Field Center X: %d\n", field_center_x);
      printf("Field Width: %d\n", field_width);
      printf("Field Height: %d\n", field_height);
      destroyAllWindows();
      return;
    }
  }
}

// Generates all the calibration prompts (field + ball + robots)
void runFullCalibration(VideoCapture capture, Ball &ball, Robot &Home1) {
  calibrateField(capture);
  calibrateBall(capture, ball);
  calibrateRobot_Home1(capture, Home1);
}

int main(int argc, char* argv[]) {
	//if we would like to calibrate our filter values, set to true.
	bool calibrationMode = true;

  // Set Initial Field Values
  field_center_x = 300;
  field_center_y = 300;
  field_width = 720;
  field_height = 400;

  int field_origin_x;
  int field_origin_y;

	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	Mat threshold;
	Mat HSV;

	//video capture object to acquire webcam feed
	const string videoStreamAddress = "http://192.168.1.126:8080/?action=stream?dummy=param.mjpg";
	VideoCapture capture;

	capture.open(1); //set to 0 to use the webcam

	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

  // When NOT in calibration mode, use actual hard-coded color values
  Robot home1(HOME), home2(HOME);
  Robot away1(AWAY), away2(AWAY);
  Ball ball;

  if (calibrationMode == true) {
    // Calibrate the camera first
    runFullCalibration(capture, ball, home1);
  }

  /************************************************************************/
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while(1) {
		//store image to matrix
		capture.read(cameraFeed);

		//convert frame from BGR to HSV colorspace
		field_origin_x = field_center_x - (field_width/2);
		field_origin_y = field_center_y - (field_height/2);
    Rect myROI(field_origin_x,field_origin_y,field_width, field_height);
    cameraFeed = cameraFeed(myROI);

		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);

    inRange(HSV,ball.getHSVmin(),ball.getHSVmax(),threshold);
    // Erode, then dialate to get a cleaner image
    morphOps(threshold);
    trackFilteredBall(ball,threshold,HSV,cameraFeed);

    inRange(HSV,home1.getHSVmin(),home1.getHSVmax(),threshold);
    // Erode, then dialate to get a cleaner image
    morphOps(threshold);
    trackFilteredRobot(home1,threshold,HSV,cameraFeed);


    // Show Field Outline
//    Rect fieldOutline(field_origin_x, field_origin_y, field_width, field_height);
//    rectangle(cameraFeed,fieldOutline,Scalar(255,255,255), 1, 8 ,0);
		imshow(windowName,cameraFeed);

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(50);
	}
	return 0;
}

