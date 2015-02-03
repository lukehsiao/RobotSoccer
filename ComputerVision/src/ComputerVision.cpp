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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robot_soccer/locations.h"


#define PI 3.14159265
#define MIN_CHANGE 5
#define MAX_CHANGE 1000

// Constants for determining field coordinate systems
#define FIELD_WIDTH 790
#define FIELD_HEIGHT 400
#define FIELD_CENTER_X 455
#define FIELD_CENTER_Y 240

using namespace cv;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

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

}
string intToString(int number) {
  std::stringstream ss;
  ss << number;
  return ss.str();
}
void createTrackbars() {
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

  field_x = img_x - FIELD_CENTER_X;

  field_y = FIELD_CENTER_Y - img_y;

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

  circle(frame,cv::Point(x,y),10,cv::Scalar(0,0,255));
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

int main(int argc, char* argv[]) {
	//if we would like to calibrate our filter values, set to true.
	bool calibrationMode = false;

	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	Mat threshold;
	Mat HSV;

	if(calibrationMode){
		//create slider bars for HSV filtering
		createTrackbars();
	}
	//video capture object to acquire webcam feed
	//VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	//capture.open("demo.mp4");

	//set height and width of capture frame
	//capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	//capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

  // When NOT in calibration mode, use actual hard-coded color values
  Robot home1(HOME), home2(HOME);
  Robot away1(AWAY), away2(AWAY);
  Ball ball;

  /***********************Ros Publisher************************************/

  ros::init(argc, argv, "computer_vision");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<robot_soccer::locations>("locTopic", 1000);
  ros::Rate loop_rate(10);

  /************************************************************************/

	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while(ros::ok()) {
		//store image to matrix
		//capture.read(cameraFeed);
		//convert frame from BGR to HSV colorspace

    system("wget -q \"http://192.168.1.222/admin-bin/ccam.cgi?opt=vxyhc&ww=2048&wh=1536\" -O image.jpg");

    // Use OpenCV to open "image.jpg" here and dump into mat
    cameraFeed = imread("image.jpg", CV_LOAD_IMAGE_COLOR);

    //Crop out stuff
    if (cameraFeed.cols < 700 || cameraFeed.rows < 600) {
      continue;
    }
    Rect myROI(60,140,920,470);
    cameraFeed = cameraFeed(myROI);

		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);

		if(calibrationMode==true) {
		  //if in calibration mode, we track objects based on the HSV slider values.
		  inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);

		  // Erode, then dialate to get a cleaner image
		  morphOps(threshold);

		  imshow(windowName2,threshold);
		  trackFilteredBall(ball,threshold,HSV,cameraFeed);
		}
		else {
		  inRange(HSV,ball.getHSVmin(),ball.getHSVmax(),threshold);
		  // Erode, then dialate to get a cleaner image
		  morphOps(threshold);
		  trackFilteredBall(ball,threshold,HSV,cameraFeed);

      inRange(HSV,home1.getHSVmin(),home1.getHSVmax(),threshold);
      // Erode, then dialate to get a cleaner image
      morphOps(threshold);
      trackFilteredRobot(home1,threshold,HSV,cameraFeed);

//      inRange(HSV,away1.getHSVmin(),away1.getHSVmax(),threshold);
//      // Erode, then dialate to get a cleaner image
//      morphOps(threshold);
//      trackFilteredRobot(away1,threshold,HSV,cameraFeed);

		}

    // Show Field Outline
    Rect fieldOutline(60, 40, 790, 400);
    rectangle(cameraFeed,fieldOutline,Scalar(255,255,255), 1, 8 ,0);
		imshow(windowName,cameraFeed);

		/***********************Ros Publisher************************************/

		// Create message object
		robot_soccer::locations coordinates;
		// Fill message object with values
		coordinates.home1_x = home1.get_x_pos();
		coordinates.home1_y = home1.get_y_pos();
		coordinates.home1_theta = home1.getAngle();
		// Print values to ROS console
		ROS_INFO("x %d\ny %d\ntheta %d\n", coordinates.home1_x, coordinates.home1_y, coordinates.home1_theta);
		// Publish message
		publisher.publish(coordinates);
		// Waits the necessary time between message publications to meet the
		// specified frequency set above (ros::Rate loop_rate(10);)
		loop_rate.sleep();

		/************************************************************************/

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(50);
	}
	return 0;
}

