//============================================================================
// Name : Robot.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : Implementation of Robot methods
//============================================================================

#include "Robot.h"

using namespace cv;

Robot::Robot(int TEAM) : Object() {
  Robot::team = TEAM;
  Robot::angle = 0;
  Robot::old_angle = 0;
  // Hard code team colors here
  if (team == HOME) {
    setHSVmin(cv::Scalar(0,0,235));
    setHSVmax(cv::Scalar(5,255,255));
  }
  else if (team == AWAY) {
    setHSVmin(cv::Scalar(103,67,0));
    setHSVmax(cv::Scalar(138,217,255));
  }
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}

// Draws human-readable markers on the image showing robot information
void Robot::drawRobot(Mat &frame) {
  int x = this->get_img_x();
  int y = this->get_img_y();
  int team = this->getTeam();
  int angle = this->getAngle();

  int real_x = this->get_x_pos();
  int real_y = this->get_y_pos();

  // TODO there may be some error in this value to do compensating for the noise
  circle(frame,cv::Point(x,y),7, Scalar(0,0,255));
  putText(frame,"(" + intToString(real_x)+ "," + intToString(real_y) + ")",
          Point(x,y+20),1,1,Scalar(0,255,0));
  putText(frame, "Robot", Point(x+17,y+35),1,1,Scalar(0,255,0));
  putText(frame, "Team " + intToString(team), Point(x+17,y+50),1,1,Scalar(0,255,0));
  putText(frame, "Angle: " + intToString(angle), Point(x+17,y+65),1,1,Scalar(0,255,0));
}

// Generate prompts to calibrate colors for this robot
void Robot::calibrateRobot(VideoCapture capture) {
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

      setHSVmin(hsv_min);
      setHSVmax(hsv_max);

      trackFilteredRobot(threshold,HSV,cameraFeed);

      imshow(windowName2,threshold);
      imshow(windowName,cameraFeed);

      char pressedKey;
      pressedKey = cvWaitKey(50); // Wait for user to press 'Enter'
      if (pressedKey == '\n') {

          Scalar hsv_min(h_min, s_min, v_min);
          Scalar hsv_max(h_max, s_max, v_max);

          setHSVmin(hsv_min);
          setHSVmax(hsv_max);

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

// Function specific for tracking robots. Will calculate the center of the robot as
// well as the it's angle in relation to the horizontal.
void Robot::trackFilteredRobot(Mat threshold, Mat HSV, Mat &cameraFeed) {
  Mat temp;
  threshold.copyTo(temp);
  morphOps(temp);

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
    circle(cameraFeed, centerPoints[c1], 6, Scalar(255,0,0), -1, 8, 0);

    //Draw line between centers
    line(cameraFeed, centerPoints[c1], centerPoints[c2], Scalar(0,0,255), 2, 8, 0);

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
    //if (intAngle > 90) {
    //  intAngle = intAngle - 90;
    //}
    //else {
    //  intAngle = 270 + intAngle;
    //}

    // Center the points of the robot
    int real_center_x;
    int real_center_y;

    real_center_x = (int)(centerPoints[c1].x + centerPoints[c2].x)/2;
    real_center_y = (int)(centerPoints[c1].y + centerPoints[c2].y)/2;

    circle(cameraFeed, Point(real_center_x, real_center_y), 3, Scalar(255,255,255), -1, 8, 0);

    Point fieldPosition = convertCoordinates(Point(real_center_x,
                                                   real_center_y));

    // Assign Robot it's variables based on team
    if (TEAM == HOME) {
      if (abs(intAngle - this->getOldAngle()) > MIN_CHANGE) {
        this->setAngle(intAngle);
      }
      if (abs(fieldPosition.x - this->get_x_pos()) > MIN_CHANGE &&
          abs(fieldPosition.x - this->get_x_pos()) < MAX_CHANGE) {
        this->set_x_pos(fieldPosition.x);
        this->set_img_x((int)centerPoints[c1].x);
      }
      if (abs(fieldPosition.y - this->get_y_pos()) > MIN_CHANGE &&
          abs(fieldPosition.y - this->get_y_pos()) < MAX_CHANGE) {
        this->set_y_pos(fieldPosition.y);
        this->set_img_y((int)centerPoints[c1].y);
      }
    }
    else {
      // Convert to Away Angle
      if (intAngle <= 180) {
        intAngle = 180 + intAngle;
      }
      else {
        intAngle = intAngle - 180;
      }
      if (abs(intAngle - this->getOldAngle()) > MIN_CHANGE) {
        this->setAngle(intAngle);
      }
      if (abs(fieldPosition.x + this->get_x_pos()) > MIN_CHANGE &&
          abs(fieldPosition.x + this->get_x_pos()) < MAX_CHANGE) {
        this->set_x_pos(-fieldPosition.x);
        this->set_img_x((int)centerPoints[c1].x);
      }
      if (abs(fieldPosition.y + this->get_y_pos()) > MIN_CHANGE &&
          abs(fieldPosition.y + this->get_y_pos()) < MAX_CHANGE) {
        this->set_y_pos(-fieldPosition.y);
        this->set_img_y((int)centerPoints[c1].y);
      }
    }

    this->drawRobot(cameraFeed);
  }
}

// Setters and Getters
void Robot::setAngle(int newAngle) {
  Robot::old_angle = Robot::angle;
  Robot::angle = newAngle;
}

int Robot::getAngle() {
  return Robot::angle;
}

int Robot::getOldAngle() {
  return Robot::old_angle;
}

// Set which team the robot is on. 1 = HOME, 2 = AWAY
void Robot::setTeam(int team) {
  Robot::team = team;
}

// Returns which team the robot is on. 1 = HOME, 2 = AWAY
int Robot::getTeam() {
  return Robot::team;
}
