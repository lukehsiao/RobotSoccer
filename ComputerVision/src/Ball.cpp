//============================================================================
// Name : Ball.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Implementation of Ball methods
//============================================================================

#include "Ball.h"

using namespace cv;

Ball::Ball() : Object() {
  setHSVmin(cv::Scalar(20,90,150));
  setHSVmax(cv::Scalar(45,186,238));
}

Ball::~Ball() {
	// TODO Auto-generated destructor stub
}

// Generates prompts for calibration of the color ball
void Ball::calibrateBall(VideoCapture capture) {
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

  // Set Trackbar intial values
  setTrackbarPos( "H_MIN", trackbarWindowName, this->getHSVmin().val[0]);
  setTrackbarPos( "H_MAX", trackbarWindowName, this->getHSVmax().val[0]);
  setTrackbarPos( "S_MIN", trackbarWindowName, this->getHSVmin().val[1]);
  setTrackbarPos( "S_MAX", trackbarWindowName, this->getHSVmax().val[1]);
  setTrackbarPos( "V_MIN", trackbarWindowName, this->getHSVmin().val[2]);
  setTrackbarPos( "V_MAX", trackbarWindowName, this->getHSVmax().val[2]);

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

    this->setHSVmin(hsv_min);
    this->setHSVmax(hsv_max);

    trackFilteredBall(threshold,HSV,cameraFeed);

    imshow(windowName,cameraFeed);
    imshow(windowName2,threshold);

    char pressedKey;
    pressedKey = cvWaitKey(50); // Wait for user to press 'Enter'
    if (pressedKey == '\n') {
       Scalar hsv_min(h_min, s_min, v_min);
       Scalar hsv_max(h_max, s_max, v_max);

       this->setHSVmin(hsv_min);
       this->setHSVmax(hsv_max);

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

// Finds the contours (outlines) of the now filtered image and determine's its
// center by examining its moments.
void Ball::trackFilteredBall(Mat threshold, Mat HSV, Mat &cameraFeed) {

  Mat temp;
  threshold.copyTo(temp);
  morphOps(temp);
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

        if (TEAM == HOME) {
          if (abs(this->get_x_pos() - fieldPosition.x) > MIN_CHANGE) {
            this->set_x_pos(fieldPosition.x);
            this->set_img_x(moment.m10/moment.m00);
          }

          if (abs(this->get_y_pos() - fieldPosition.y) > MIN_CHANGE) {
            this->set_y_pos(fieldPosition.y);
            this->set_img_y(moment.m01/moment.m00);
          }
        }
        else {
          if (abs(this->get_x_pos() + fieldPosition.x) > MIN_CHANGE) {
            this->set_x_pos(-fieldPosition.x);
            this->set_img_x(moment.m10/moment.m00);
          }

          if (abs(this->get_y_pos() + fieldPosition.y) > MIN_CHANGE) {
            this->set_y_pos(-fieldPosition.y);
            this->set_img_y(moment.m01/moment.m00);
          }
        }

        objectFound = true;

      }
      else {
        objectFound = false;
      }

      //let user know you found an object
      if(objectFound == true){
        //draw object location on screen
        this->drawBall(cameraFeed);
      }

    }
    else {
      putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
    }
  }
}

// Draws the ball location and information
void Ball::drawBall(Mat &frame) {
  int x = this->get_img_x();
  int y = this->get_img_y();

  int real_x = this->get_x_pos();
  int real_y = this->get_y_pos();
  circle(frame,cv::Point(x,y),10,cv::Scalar(0,0,255));
  putText(frame,"(" + intToString(real_x)+ "," + intToString(real_y) + ")",
          Point(x,y+20),1,1,Scalar(0,255,0));
  putText(frame, "Ball", Point(x+25,y+35),1,1,Scalar(0,255,0));
}
