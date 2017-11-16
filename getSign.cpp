#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "/home/pi/OpenNI2/Samples/SimpleRead/AstraManager.h"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

AstraManager Astra;
openni::DepthPixel* depth;

Mat roadSign;

bool getSign(VideoCapture& cap) {
	Mat imgOriginal, imgHSV;
	int findCircleCnt = 0;
	bool bSuccess = cap.read(imgOriginal); // read a new frame from video
    if (!bSuccess) {//if not success, break loop 
        cout << "Cannot read a frame from video stream" << endl;
        return false;
    }
    //namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
    //namedWindow( "Thresholded Image", CV_WINDOW_AUTOSIZE );
    for(int i = 0; i < 15; i++) {
    	bool bSuccess = cap.read(imgOriginal); // read a new frame from video
        if (!bSuccess) {//if not success, break loop
            cout << "Cannot read a frame from video stream" << endl;
            return false;
        }
        imgOriginal.convertTo(imgOriginal, CV_32FC3, 1.0/255.0);
        Scalar m;
        m=cv::mean(imgOriginal);
        imgOriginal-=m;
        imgOriginal+=Scalar(0.3,0.3,0.3); // Changing this you can adjust color balance.
		normalize(imgOriginal,imgOriginal,0,1,cv::NORM_MINMAX);
		imgOriginal.convertTo(imgOriginal, CV_8UC3, 255.0);

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        //cvtColor( imgOriginal, src_gray, CV_BGR2GRAY ); //Convert the captured frame from BGR to GRAY
	
		Mat mask1, mask2, mask3;
        inRange(imgHSV, Scalar(0, 70, 80), Scalar(15, 255, 255), mask1);
        inRange(imgHSV, Scalar(165, 70, 80), Scalar(180, 255, 255), mask2);
        inRange(imgHSV, Scalar(90, 70, 80), Scalar(150, 255, 255), mask3);
	
        Mat imgThresholded = mask1 | mask2 | mask3;

        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        vector<Vec3f> circles;

        /// Apply the Hough Transform to find the circles
        HoughCircles( imgThresholded, circles, CV_HOUGH_GRADIENT, 1, imgThresholded.cols, 25, 20, 20, 0 );
        cout << "found " << circles.size() << " circles" << endl;
        int height = imgOriginal.rows;
        int width = imgOriginal.cols;
        /// Draw the circles detected
        for( size_t i = 0; i < circles.size(); i++ ) {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            if (center.x + 2*radius > width or center.x - 2*radius < 0 or center.y + 2*radius > height or center.y - 2*radius < 0) {
                continue;
            }
            // circle center
            //circle( imgOriginal, center, 3, 255, -1, 8, 0 );
            // circle outline
            //circle( imgOriginal, center, radius, 255, 3, 8, 0 );
            
            Rect rect(center.x - 2 * radius, center.y - 2 * radius, 4 * radius, 4 * radius);

            Mat cropped;
            imgOriginal(rect).copyTo(cropped);
            imwrite("test.ppm", cropped);
	    	if(findCircleCnt >= 2) {
                roadSign = cropped;
                return true;
            }
	    	findCircleCnt++;
        }
        //imshow("Thresholded Image", imgThresholded); //show the thresholded image
        //imshow( "Hough Circle Transform Demo", imgOriginal );
        waitKey(100);
    }
    return false;
}

/** @function main */
int main(int argc, char** argv)
{

    VideoCapture cap(0); //capture the video from web cam
    
    if ( !cap.isOpened() )  // if not success, exit program
    {
        cout << "Cannot open the web cam" << endl;
        return -1;
    }
    
    while(true) {
	    if(getSign(cap)) cout << "Yes we get a new sign" << endl;
	    else cout << "No we cannot get new sign" << endl;
	}
    
    return 0;
}