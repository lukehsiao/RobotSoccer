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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv/cv.h>
#include "Ball.h"
#include "Robot.h"
#include "Object.h"

#define PI 3.14159265
#define MIN_CHANGE 5
#define MAX_CHANGE 1000

//default capture width and height
#define FRAME_WIDTH  1280
#define FRAME_HEIGHT 720

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
int field_height_max = FRAME_HEIGHT;
int field_width_max = FRAME_WIDTH;
int field_center_x_max = FRAME_WIDTH;
int field_center_y_max = FRAME_HEIGHT;

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

// Camera Calibration Data
float dist_coeff[5][1] = {  {-2.0698033501549058},
                            {9.6448611626711713},
                            {0.0},
                            {0.0},
                            {-20.765851606846589}
                         };

float cam_matrix[3][3] = {  {1514.8407346906349,0.0,639.5},
                            {0.0,1514.8407346906349,359.5},
                            {0.0,0.0,1.0}
                         };

// This function is called whenever a trackbar changes
void on_trackbar( int, void* ) {
  // Does nothing
}

string intToString(int number) {
  std::stringstream ss;
  ss << number;
  return ss.str();
}

// Runs the undistortion
void undistortImage(Mat &source) {
  // Setup Distortion matrices
  Mat cameraMatrix = Mat(3, 3, CV_32FC1, &cam_matrix);
  Mat distCoeffs = Mat(5, 1, CV_32FC1, &dist_coeff);

  Size imageSize = source.size();
  Mat temp = source.clone();
  undistort(temp, source, cameraMatrix, distCoeffs,
                  getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0));
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

  field_x = img_x - (field_width/2);

  field_y = (field_height/2) - img_y;

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
  Mat dilateElement = getStructuringElement( MORPH_RECT,Size(3,3));

  erode(thresh,thresh,erodeElement);
  dilate(thresh,thresh,dilateElement);

  erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
  //dilate with larger element so make sure object is nicely visible
  dilateElement = getStructuringElement( MORPH_RECT,Size(6,6));

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

    // Get momentsf
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

    // Correct angle to the Robot's X-axis
    if (intAngle > 90) {
      intAngle = intAngle - 90;
    }
    else {
      intAngle = 360 - intAngle;
    }


    // TODO Filtering bad data (if the change in xpos or ypos is too large, ignore data
    // Set Robot variables

    if (abs(intAngle - robot.getOldAngle()) > 5) {
      robot.setAngle(intAngle);
    }

    Point fieldPosition = convertCoordinates(Point((int)centerPoints[c1].x,
                                                   (int)centerPoints[c1].y));
    if (abs(fieldPosition.x - robot.get_x_pos()) > MIN_CHANGE &&
        abs(fieldPosition.x - robot.get_x_pos()) < MAX_CHANGE) {
      robot.set_x_pos(fieldPosition.x);
      robot.set_img_x((int)centerPoints[c1].x);
    }
    if (abs(fieldPosition.y - robot.get_y_pos()) > MIN_CHANGE &&
        abs(fieldPosition.y - robot.get_y_pos()) < MAX_CHANGE) {
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
  int largest_area = 0;
  int largest_contour_index = 0;

  //these two vectors needed for output of findContours
  vector< vector<Point> > contours;
  vector<Vec4i> hierarchy;

  //find contours of filtered image using openCV findContours function
  findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

  //use moments method to find our filtered object
  //double refArea = 0;
  bool objectFound = false;

  if (contours.size() > 0) {
    int numObjects = hierarchy.size();
    //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
    if(numObjects<MAX_NUM_OBJECTS){

      // Iterate over all contours to find the largest
      for (unsigned i = 0; i < contours.size(); i++) {
        double tempArea = contourArea(contours[i], false); // Find area of each contour
        if (tempArea > largest_area) {
          largest_area = tempArea;
          largest_contour_index = i;
        }
      }

      Moments moment = moments((Mat)contours[largest_contour_index]);

      //if the area is less than 20 px by 20px then it is probably just noise
      //if the area is the same as the 3/2 of the image size, probably just a bad filter
      //we only want the object with the largest area so we safe a reference area each
      //iteration and compare it to the area in the next iteration.
      if(largest_area > MIN_OBJECT_AREA) {
        Point fieldPosition = convertCoordinates(Point(moment.m10/moment.m00,
                                                       moment.m01/moment.m00));

        if (abs(ball.get_x_pos() - fieldPosition.x) > MIN_CHANGE) {
          ball.set_x_pos(fieldPosition.x);
          ball.set_img_x(moment.m10/moment.m00);
        }

        if (abs(ball.get_y_pos() - fieldPosition.y) > MIN_CHANGE) {
          ball.set_y_pos(fieldPosition.y);
          ball.set_img_y(moment.m01/moment.m00);
        }

        objectFound = true;

      }
      else {
        objectFound = false;
      }

      //let user know you found an object
      if(objectFound == true){
        //draw object location on screen
        drawBall(ball,cameraFeed);
      }

    }
    else {
      putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
    }
  }
}

// Generate prompts to calibrate colors for the Home1 robots
void calibrateRobot(VideoCapture capture, Robot &robot) {
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

  // Set Trackbar intial values to near Red
  setTrackbarPos( "H_MIN", trackbarWindowName, 0);
  setTrackbarPos( "H_MAX", trackbarWindowName, 15);
  setTrackbarPos( "S_MIN", trackbarWindowName, 0);
  setTrackbarPos( "S_MAX", trackbarWindowName, 255);
  setTrackbarPos( "V_MIN", trackbarWindowName, 195);
  setTrackbarPos( "V_MAX", trackbarWindowName, 255);

  // Wait forever until user sets the values
   while (1) {
      //store image to matrix
      capture.read(cameraFeed);
      undistortImage(cameraFeed);

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

      robot.setHSVmin(hsv_min);
      robot.setHSVmax(hsv_max);

      trackFilteredRobot(robot,threshold,HSV,cameraFeed);

      imshow(windowName2,threshold);
      imshow(windowName,cameraFeed);

      char pressedKey;
      pressedKey = cvWaitKey(50); // Wait for user to press 'Enter'
      if (pressedKey == '\n') {

          Scalar hsv_min(h_min, s_min, v_min);
          Scalar hsv_max(h_max, s_max, v_max);

          robot.setHSVmin(hsv_min);
          robot.setHSVmax(hsv_max);

          printf("\n\nRobot HSV Values Saved!\n");
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
  setTrackbarPos( "H_MIN", trackbarWindowName, 30);
  setTrackbarPos( "H_MAX", trackbarWindowName, 65);
  setTrackbarPos( "S_MIN", trackbarWindowName, 0);
  setTrackbarPos( "S_MAX", trackbarWindowName, 255);
  setTrackbarPos( "V_MIN", trackbarWindowName, 180);
  setTrackbarPos( "V_MAX", trackbarWindowName, 255);

  // Wait forever until user sets the values
   while (1) {
    //store image to matrix
    capture.read(cameraFeed);
    undistortImage(cameraFeed);

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

  // Set Initial Field Values
  field_center_x = 623;
  field_center_y = 356;
  field_width = 828;
  field_height = 562;

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
    undistortImage(cameraFeed);

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
void runFullCalibration(VideoCapture capture, Ball &ball, Robot &Home1, Robot &Home2, Robot &Away1, Robot &Away2) {
  calibrateField(capture);
  calibrateBall(capture, ball);
  calibrateRobot(capture, Home1);
  calibrateRobot(capture, Away1);
}

Mat OR(Mat mat1, Mat mat2){
  Mat BGR_ball; //BGR image of ball;
  Mat BGR_robot; //BGR image of robot;
  Mat bw_ball; //Black and white image of ball
  Mat bw_robot;
  Mat mat3_bw;
  Mat mat3_BGR;
  Mat mat3_HSV;

  cvtColor(mat1,BGR_ball,COLOR_HSV2BGR);
  cvtColor(BGR_ball,bw_ball,COLOR_BGR2GRAY);
  cvtColor(mat2,BGR_robot,COLOR_HSV2BGR);
  cvtColor(BGR_robot,bw_robot,COLOR_BGR2GRAY);
  mat3_bw=bw_ball;

  int col=mat1.cols;
  int row=mat1.rows;

  for (int i=0; i<col; i++)
  {
    for (int j=0; j<row; j++)
    {
      mat3_bw.at<uchar>(i,j)=bw_ball.at<uchar>(i,j)+bw_robot.at<uchar>(i,j);  //assume white=255,black=0;
      if (mat3_bw.at<uchar>(i,j)>255)                     //b+b=0=b; w+b=255=w; w+w=510,over!
        mat3_bw.at<uchar>(i,j)=255;
    }// not sure is 'MAT(i,j)' the right way to
  }
  cvtColor(mat3_bw,mat3_BGR,COLOR_GRAY2BGR);
  cvtColor(mat3_BGR,mat3_HSV,COLOR_BGR2HSV);
  printf("\n\nYES!!!\n");
  return mat3_HSV;
}


int main(int argc, char* argv[]) {
	//if we would like to calibrate our filter values, set to true.
	bool calibrationMode = true;

  int field_origin_x;
  int field_origin_y;

	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	Mat threshold1; //threshold image of ball
	Mat threshold2; //threshold image of robot
	Mat threshold; //combined image
	Mat HSV;
	Mat bw; // black and white mat
  Mat BGR;// BGR mat

	//video capture object to acquire webcam feed
	const string videoStreamAddress = "http://192.168.1.90/mjpg/video.mjpg";
	VideoCapture capture;

  capture.open(videoStreamAddress); //set to 0 to use the webcam

	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

  // When NOT in calibration mode, use actual hard-coded color values
  Robot home1(HOME), home2(HOME);
  Robot away1(AWAY), away2(AWAY);
  Ball ball;

  if (calibrationMode == true) {
    // Calibrate the camera first
    runFullCalibration(capture, ball, home1, home2, away1, away2);
  }

  /************************************************************************/
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while(1) {
	  //TODO (Clover) There are bugs in your code that we'll need to fix later.
//		//store image to matrix
//		capture.read(cameraFeed);
//
//		//convert frame from BGR to HSV colorspace
//		field_origin_x = field_center_x - (field_width/2);
//		field_origin_y = field_center_y - (field_height/2);
//    Rect myROI(field_origin_x,field_origin_y,field_width, field_height);
//    cameraFeed = cameraFeed(myROI);
//
//		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
//
//    inRange(HSV,ball.getHSVmin(),ball.getHSVmax(),threshold1);
//
//    // Erode, then dialate to get a cleaner image
//    morphOps(threshold1);
//    trackFilteredBall(ball,threshold1,HSV,cameraFeed);
//
//    inRange(HSV,home1.getHSVmin(),home1.getHSVmax(),threshold2);
//    // Erode, then dialate to get a cleaner image
//    morphOps(threshold2);
//    trackFilteredRobot(home1,threshold2,HSV,cameraFeed);
//
//    // Display the filtered robot/ball images
//    //threshold=OR(threshold2,threshold2);
//
//    // Show Field Outline
//    Rect fieldOutline(0, 0, field_width-1, field_height-1);
//    rectangle(cameraFeed,fieldOutline,Scalar(255,255,255), 1, 8 ,0);
//		imshow(windowName,cameraFeed);
//    imshow(windowName2,threshold);

    //store image to matrix
    capture.read(cameraFeed);
    undistortImage(cameraFeed);

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
    Rect fieldOutline(0, 0, field_width, field_height);
    rectangle(cameraFeed,fieldOutline,Scalar(255,255,255), 1, 8 ,0);
    imshow(windowName,cameraFeed);

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(50);
	}
	return 0;
}

