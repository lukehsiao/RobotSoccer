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


#include "ComputerVision.h"
#include "Ball.h"
#include "Robot.h"
#include "Object.h"


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
int field_height_max = FRAME_HEIGHT + 100;
int field_width_max = FRAME_WIDTH + 100;
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
double dist_coeff[5][1] = {  {-0.43},
                             {0.23},
                             {-0.004},
                             {-0.005},
                             {-0.07}
                           };


double cam_matrix[3][3] = {  {846,  0.0,  639.5},
                             {0.0,  849,  436.0},
                             {0.0,  0.0,  1.0}
                          };

// Declaration of the 5 objects
Robot home1(HOME), home2(HOME);
Robot away1(AWAY), away2(AWAY);
Ball ball;

// Saves all of the pertinent calibration settings to human-readable file
void saveSettings() {
  // Open file
  std::ofstream of("settings.data");

  if (!of.is_open()) {
    printf("\n\n\nERROR OPENING SETTINGS FILE!\n\n\n");
    return;
  }

  const string h1 = "Home1";
  const string h2 = "Home2";
  const string a1 = "Away1";
  const string a2 = "Away2";
  const string b = "Ball";
  const string f = "Field";
  int tempH, tempS, tempV;

  // Robot Save format:
  // [RobotName] [Hmin] [Smin] [Vmin] [Hmax] [Smax] [Vmax]
  tempH = home1.getHSVmin().val[0];
  tempS = home1.getHSVmin().val[1];
  tempV = home1.getHSVmin().val[2];
  of << h1 << " " <<  tempH << " " <<  tempS << " " <<  tempV;
  tempH = home1.getHSVmax().val[0];
  tempS = home1.getHSVmax().val[1];
  tempV = home1.getHSVmax().val[2];
  of << " " <<  tempH << " " <<  tempS << " " <<  tempV << "\n";

  tempH = home2.getHSVmin().val[0];
  tempS = home2.getHSVmin().val[1];
  tempV = home2.getHSVmin().val[2];
  of << h2 << " " <<  tempH << " " <<  tempS << " " <<  tempV;
  tempH = home2.getHSVmax().val[0];
  tempS = home2.getHSVmax().val[1];
  tempV = home2.getHSVmax().val[2];
  of << " " <<  tempH << " " <<  tempS << " " <<  tempV << "\n";

  tempH = away1.getHSVmin().val[0];
  tempS = away1.getHSVmin().val[1];
  tempV = away1.getHSVmin().val[2];
  of << a1 << " " <<  tempH << " " <<  tempS << " " <<  tempV;
  tempH = away1.getHSVmax().val[0];
  tempS = away1.getHSVmax().val[1];
  tempV = away1.getHSVmax().val[2];
  of << " " <<  tempH << " " <<  tempS << " " <<  tempV << "\n";

  tempH = away2.getHSVmin().val[0];
  tempS = away2.getHSVmin().val[1];
  tempV = away2.getHSVmin().val[2];
  of << a2 << " " <<  tempH << " " <<  tempS << " " <<  tempV;
  tempH = away2.getHSVmax().val[0];
  tempS = away2.getHSVmax().val[1];
  tempV = away2.getHSVmax().val[2];
  of << " " <<  tempH << " " <<  tempS << " " <<  tempV << "\n";


  // Ball Save format:
  // [Ball] [Hmin] [Smin] [Vmin] [Hmax] [Smax] [Vmax]
  tempH = ball.getHSVmin().val[0];
  tempS = ball.getHSVmin().val[1];
  tempV = ball.getHSVmin().val[2];
  of << b << " " <<  tempH << " " << tempS << " " <<  tempV;
  tempH = ball.getHSVmax().val[0];
  tempS = ball.getHSVmax().val[1];
  tempV = ball.getHSVmax().val[2];
  of << " " <<  tempH << " " <<  tempS << " " <<  tempV << "\n";

  // Field Save format:
  // [Field] [x_pos] [y_pos] [width] [height]
  of << f << " " << field_center_x << " " << field_center_y;
  of << " " << field_width << " " <<  field_height << "\n";

  // close file
  of.close();
  printf("Settings Saved!\n");
}

// Reads in saved settings from settings.data and stores them in objects
void restoreSettings() {
  const string h1 = "Home1";
  const string h2 = "Home2";
  const string a1 = "Away1";
  const string a2 = "Away2";
  const string b = "Ball";
  const string f = "Field";
  int MAX_CHARS = 256;
  int tempH, tempS, tempV;

  std::stringstream ss;
  std::ifstream in("settings.data");
  if (!in.good()) {
    printf("\n\n\nERROR, Couldn't open file\n\n\n");
    return;
  }

  char line[MAX_CHARS];
  char ID[MAX_CHARS];

  //Begin Parsing File line by line
  in.getline(line, MAX_CHARS);
  ss.str(line);
  // Read Identifier
  ss >> ID;
  if (ID == h1) {
    ss >> tempH;
    ss >> tempS;
    ss >> tempV;
    home1.setHSVmin(Scalar(tempH, tempS, tempV));
    ss >> tempH;
    ss >> tempS;
    ss >> tempV;
    home1.setHSVmax(Scalar(tempH, tempS, tempV));
  }
  ss.clear();

  in.getline(line, MAX_CHARS);
  ss.str(line);
  // Read Identifier
  ss >> ID;
  if (ID == h2) {
    ss >> tempH;
    ss >> tempS;
    ss >> tempV;
    home2.setHSVmin(Scalar(tempH, tempS, tempV));
    ss >> tempH;
    ss >> tempS;
    ss >> tempV;
    home2.setHSVmax(Scalar(tempH, tempS, tempV));
  }
  ss.clear();

  in.getline(line, MAX_CHARS);
  ss.str(line);
  // Read Identifier
  ss >> ID;
  if (ID == a1) {
    ss >> tempH;
    ss >> tempS;
    ss >> tempV;
    away1.setHSVmin(Scalar(tempH, tempS, tempV));
    ss >> tempH;
    ss >> tempS;
    ss >> tempV;
    away1.setHSVmax(Scalar(tempH, tempS, tempV));
  }
  ss.clear();

  in.getline(line, MAX_CHARS);
  ss.str(line);
  // Read Identifier
  ss >> ID;
  if (ID == a2) {
    ss >> tempH;
    ss >> tempS;
    ss >> tempV;
    away2.setHSVmin(Scalar(tempH, tempS, tempV));
    ss >> tempH;
    ss >> tempS;
    ss >> tempV;
    away2.setHSVmax(Scalar(tempH, tempS, tempV));
  }
  ss.clear();

  // scan in ball
  in.getline(line, MAX_CHARS);
  ss.str(line);
  // Read Identifier
  ss >> ID;
  if (ID == b) {
    ss >> tempH;
    ss >> tempS;
    ss >> tempV;
    ball.setHSVmin(Scalar(tempH, tempS, tempV));
    ss >> tempH;
    ss >> tempS;
    ss >> tempV;
    ball.setHSVmax(Scalar(tempH, tempS, tempV));
  }
  ss.clear();

  in.getline(line, MAX_CHARS);
  ss.str(line);
  // Read Identifier
  ss >> ID;
  if (ID == f) {
    ss >> field_center_x;
    ss >> field_center_y;
    ss >> field_width;
    ss >> field_height;
  }
  ss.clear();

  in.close();
  printf("Settings Restored!\n");
}

//-----------------------------------------------------------------------------
// Utility Functions Shared by Classes
//-----------------------------------------------------------------------------

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
  Mat cameraMatrix = Mat(3, 3, CV_64F, cam_matrix); // read in 64-bit doubles
  Mat distCoeffs = Mat(5, 1, CV_64F, dist_coeff);

  double y_shift = 70;
  int enlargement = 100;
  Size imageSize = source.size();
  imageSize.height += enlargement;
  Mat temp = source.clone();

  Mat newCameraMatrix = cameraMatrix.clone();

  // Adjust the position of the newCameraMatrix
  // at() is a templated function so <double> is necessary
  newCameraMatrix.at<double>(1,2) += y_shift; //shift the image down

  Mat map1;
  Mat map2;
  initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), newCameraMatrix, imageSize, CV_16SC2, map1, map2);

  remap(temp, source, map1, map2, INTER_LINEAR);
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
//-----------------------------------------------------------------------------
//  End of Utility Functions Shared by Classes
//-----------------------------------------------------------------------------

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
  restoreSettings();
  calibrateField(capture);
  ball.calibrateBall(capture);
  Home1.calibrateRobot(capture);
  //Home2.calibrateRobot(capture);
  Away1.calibrateRobot(capture);
  //Away2.calibrateRobot(capture);
  saveSettings();
}

// TODO (Clover Wu) This function is supposed to OR together the processed MATS
// To produce a balck and white image illustrating how well our color parsing
// is performing.
//Mat OR(Mat mat1, Mat mat2){
//  Mat BGR_ball; //BGR image of ball;
//  Mat BGR_robot; //BGR image of robot;
//  Mat bw_ball; //Black and white image of ball
//  Mat bw_robot;
//  Mat mat3_bw;
//  Mat mat3_BGR;
//  Mat mat3_HSV;
//
//  cvtColor(mat1,BGR_ball,COLOR_HSV2BGR);
//  cvtColor(BGR_ball,bw_ball,COLOR_BGR2GRAY);
//  cvtColor(mat2,BGR_robot,COLOR_HSV2BGR);
//  cvtColor(BGR_robot,bw_robot,COLOR_BGR2GRAY);
//  mat3_bw=bw_ball;
//
//  int col=mat1.cols;
//  int row=mat1.rows;
//
//  for (int i=0; i<col; i++)
//  {
//    for (int j=0; j<row; j++)
//    {
//      mat3_bw.at<uchar>(i,j)=bw_ball.at<uchar>(i,j)+bw_robot.at<uchar>(i,j);  //assume white=255,black=0;
//      if (mat3_bw.at<uchar>(i,j)>255)                     //b+b=0=b; w+b=255=w; w+w=510,over!
//        mat3_bw.at<uchar>(i,j)=255;
//    }// not sure is 'MAT(i,j)' the right way to
//  }
//  cvtColor(mat3_bw,mat3_BGR,COLOR_GRAY2BGR);
//  cvtColor(mat3_BGR,mat3_HSV,COLOR_BGR2HSV);
//  printf("\n\nYES!!!\n");
//  return mat3_HSV;
//}

// Special function for both getting the next image and reading the timestamp
// from the IP camera. Currently NOT very optimized for performance.
Time getNextImage(std::ifstream & myFile, std::vector<char> & imageArray) {
  imageArray.clear();
  char buffer[4];
  Time timestamp;
  bool foundImage = false;
  while(!foundImage){
    myFile.read(buffer,1);
    if((*buffer) == (char)0xFF){
      myFile.read(buffer,1);
      if((*buffer) == (char)0xD8){
        //printf("found start of image \n");
        imageArray.push_back((char)0xFF);
        imageArray.push_back((char)0xD8);
        while(1){
          myFile.read(buffer,1);
          imageArray.push_back(*buffer);
          if((*buffer) == (char)0xFF){
            myFile.read(buffer,1);
            imageArray.push_back(*buffer);
            if((*buffer) == (char)0xFE){
              myFile.read(buffer,4);
              imageArray.push_back(*buffer);
              imageArray.push_back(*(buffer+1));
              imageArray.push_back(*(buffer+2));
              imageArray.push_back(*(buffer+3));
              if((*(buffer+3)) == (char)0x01) {
                myFile.read(buffer,4);
                unsigned int sec = 0;
                for(int i = 0; i < 4; i++){
                  imageArray.push_back(*(buffer + i));
                  sec <<= 8;
                  sec += *(unsigned char*)(void*)(buffer+i);
                }

                myFile.read(buffer,1);
                unsigned int hundreds = 0;
                imageArray.push_back(*buffer);
                hundreds += *(unsigned char*)(void*)buffer;
                timestamp.sec = sec;
                timestamp.nsec = hundreds * 10000000;
              }


            }else if((*buffer) == (char)0xD9){
              //printf("found end of image\n");
              foundImage = true;
              break;
            }
          }
        }
      }
    }
  }
  return timestamp;
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

  // Set Initial Field Values
  field_center_x = 590;
  field_center_y = 410;
  field_width = 1074;
  field_height = 780;

	//video capture object to acquire webcam feed
	const string videoStreamAddress = "http://192.168.1.90/mjpg/video.mjpg";

	VideoCapture capture;

  capture.open(videoStreamAddress); //set to 0 to use the webcam

	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

  if (calibrationMode == true) {
    // Calibrate the camera first
    runFullCalibration(capture, ball, home1, home2, away1, away2);
  }

  namedWindow(windowName,WINDOW_NORMAL);

  /************************************************************************/
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
  capture.release();
  system("curl -s http://192.168.1.90/mjpg/video.mjpg > imagefifo &");
  std::ifstream myFile ("imagefifo",std::ifstream::binary);
  std::vector<char> imageArray;
	while(1) {

    //store image to matrix
    //capture.read(cameraFeed);
	  Time timestamp = getNextImage(myFile,imageArray);
	  printf("%u.%09u\n",timestamp.sec,timestamp.nsec);
	  cameraFeed = imdecode(imageArray,CV_LOAD_IMAGE_COLOR);
    undistortImage(cameraFeed);

    //convert frame from BGR to HSV colorspace
    field_origin_x = field_center_x - (field_width/2);
    field_origin_y = field_center_y - (field_height/2);
    Rect myROI(field_origin_x,field_origin_y,field_width, field_height);
    cameraFeed = cameraFeed(myROI);

    cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);

    // Track Ball
    inRange(HSV,ball.getHSVmin(),ball.getHSVmax(),threshold);
    ball.trackFilteredBall(threshold,HSV,cameraFeed);

    // Track Home 1
    inRange(HSV,home1.getHSVmin(),home1.getHSVmax(),threshold);
    home1.trackFilteredRobot(threshold,HSV,cameraFeed);

    // Track Away 1
    inRange(HSV,away1.getHSVmin(),away1.getHSVmax(),threshold);
    away1.trackFilteredRobot(threshold,HSV,cameraFeed);


    // Show Field Outline
    Rect fieldOutline(0, 0, field_width, field_height);
    rectangle(cameraFeed,fieldOutline,Scalar(255,255,255), 1, 8 ,0);
    //create window for trackbars
    imshow(windowName,cameraFeed);

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(1);
	}
	return 0;
}

