// Compiles with:
// g++ `pkg-config --cflags opencv` webcam_undistort.cpp -o webcam_undistort 
//     `pkg-config --libs opencv`

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;


volatile int quit_signal=0;
#ifdef __unix__
#include <signal.h>
extern "C" void quit_signal_handler(int signum) {
 if (quit_signal!=0) exit(0); // just exit already
 quit_signal=1;
 printf("Will quit at next camera frame (repeat to kill now)\n");
}
#endif

int main()
{
	int numBoards = 0;
	int numCornersHor;
	int numCornersVer;

  #ifdef __unix__
     signal(SIGINT,quit_signal_handler); // listen for ctrl-C
  #endif

	printf("Enter number of corners along width: ");
	scanf("%d", &numCornersHor);

	printf("Enter number of corners along height: ");
	scanf("%d", &numCornersVer);

	printf("Enter number of boards: ");
	scanf("%d", &numBoards);

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);

	vector < vector < Point3f > >object_points;
	vector < vector < Point2f > >image_points;

	vector < Point2f > corners;
	int successes = 0;

	Mat image;
	Mat gray_image;
	
  //video capture object to acquire webcam feed
	const string videoStreamAddress = "http://192.168.1.90/mjpg/video.mjpg";
	VideoCapture capture;

  capture.open(videoStreamAddress); //set to 0 to use the webcam
  
  //set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,1280);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,720);
	
	//store image to matrix
  capture.read(image);

	vector < Point3f > obj;
	for (int j = 0; j < numSquares; j++)
		obj.push_back(Point3f
			      (j / numCornersHor, j % numCornersHor, 0.0f));

	while (successes < numBoards) {

		cvtColor(image, gray_image, CV_BGR2GRAY);
		bool found = findChessboardCorners(image, board_sz, corners,
						   CALIB_CB_ADAPTIVE_THRESH |
						   CALIB_CB_FILTER_QUADS);

		if (found) {
			cornerSubPix(gray_image, corners, Size(11, 11),
				     Size(-1, -1),
				     TermCriteria(CV_TERMCRIT_EPS |
						  CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_sz, corners,
					      found);
		}

		imshow("win1", image);
		imshow("win2", gray_image);

		capture.read(image);
    if (quit_signal) exit(0); // exit cleanly on interrupt
		
		int key = waitKey(50);

		if (key == 27) {
			return 0;
		}

		if (key == ' ' && found != 0) {
			image_points.push_back(corners);
			object_points.push_back(obj);

			printf("Snap stored!");

			successes++;

			if (successes >= numBoards)
				break;
		}
	}
	
	// TODO: Output these into XML using OpenCV
	Mat intrinsic = Mat(3, 3, CV_32FC1);
	Mat distCoeffs;
	vector < Mat > rvecs;
	vector < Mat > tvecs;

	intrinsic.ptr < float >(0)[0] = 1;
	intrinsic.ptr < float >(1)[1] = 1;

	calibrateCamera(object_points, image_points, image.size(), intrinsic,
			distCoeffs, rvecs, tvecs);

	Mat imageUndistorted;
	while (1) {
		capture.read(image);
    if (quit_signal) exit(0); // exit cleanly on interrupt 
		undistort(image, imageUndistorted, intrinsic, distCoeffs);

		imshow("win1", image);
		imshow("win2", imageUndistorted);
		waitKey(1);
	}
	capture.release();

	return 0;
}

