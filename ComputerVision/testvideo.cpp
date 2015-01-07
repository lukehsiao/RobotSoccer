#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    VideoCapture cap(0); // open video camera (no. 0)

    if (!cap.isOpened())  // if open failed, exit program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);   //get frame width
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get frame height

    cout << "Frame size : " << dWidth << " x " << dHeight << endl;

    namedWindow("Demo",CV_WINDOW_AUTOSIZE); //create windowa

    Mat edges, edges2;
    while (1)
    {
        Mat frame;
        bool bSuccess = cap.read(frame); // read new frame (BGR)

	if (!bSuccess) // if read fails, break loop
        {
	    cout << "Cannot read a frame from video stream" << endl;
	    break;
        }

	// experiment with the following by removing the leading // 
	// from the lines in any given section

	// #0
	// show the original video stream
        // imshow("Demo", frame); // show the frame in "Demo" window

	// #1
	// make the image smaller, blur it, display blurred image
	//resize( frame, edges, Size(), 0.4, 0.4, INTER_LINEAR);
	//blur( edges, edges2, Size(31,31), Point(-1,1), BORDER_DEFAULT);
	//imshow("Demo", edges2);

	// #2
	// convert to gray-scale, blur a little, then detect edges
	//cvtColor(frame, edges, CV_BGR2GRAY);
	//GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
	//Canny(edges, edges, 0, 30, 3);
        //imshow("Demo", edges); // show the frame in "Demo" window

	// #3   BGR thresholding
	// try thresholding functionality
	// experiment with ranges of BGR values to identify red 
	// pixels in image (Not so easy!)
	//inRange( frame, Scalar(0,0,120), Scalar(100,100,255), edges);
	//imshow("Demo", edges);

	// #4   HSV thresholding
	// convert to HSV color space, look for red pixels (with hue 
	// values from 0-20 and 160-179)  (Also not so easy!)
	//Mat dst1, dst2;
	//cvtColor( frame, edges, CV_BGR2HSV, 3 );
	//inRange( edges, Scalar(0,90,90), Scalar(19,255,255), dst1);
	//inRange( edges, Scalar(160,90,90), Scalar(179,255,255), dst2);
	//bitwise_or( dst1, dst2, edges2);
	//imshow("Demo", edges2);
    }
    return 0;

}
