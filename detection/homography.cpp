/*
 * This file contains every function required to compute the relationship between what the camera sees
 * and a 2D grid
 * 
 * call getHomography(VideoCapture camera) will return the homography as a 3x3 Mat
 *  
 */

#include "homography.h"

using namespace cv;

struct Data {
    Mat I1,I2;
    vector<Point2f> pointsI1;
    vector<Point2f> pointsI2;
    int counterI1, counterI2;
    Mat H;
};

/*
 * Given x and y on a webcam image, this will return the position on the grid using homography H
 */
Point2f computePositionOnGrid(float x, float y, Mat H) {
    Mat_<double> pointI1(3,1);
    pointI1(0,0) = x;
    pointI1(1,0) = y;
    pointI1(2,0) = 1;
    
    Mat_<double> pointI2 = H*pointI1; // matrix multiplication
    
    // pointI2 = (X Y l)
    // true coordinates are given by x=X/l and y=Y/l
    return Point2f(pointI2.at<double>(0,0) / pointI2.at<double>(2,0), pointI2.at<double>(1,0) / pointI2.at<double>(2,0));
}

/*
 * Handles a click on the webcam image
 */
void onMouseImage(int event, int x, int y, int foo, void* data) {
    if (event!= CV_EVENT_LBUTTONDOWN) // we just take click event into account
		return;
    Data * D = ((Data*) data);
    Mat I = D->I1;
    
    if (D->counterI1 < 5) { // if we have less than five points, then we had the point to our list and draw a circle on the image
        D->pointsI1.push_back(Point(x,y));
        D->counterI1 ++;
        Point2f m(x, y);
        circle(I, m, 6, Scalar(0, 0, 255), 2); 
        imshow("image", I);
    }
}

/*
 * Handles a click on the grid image
 */
void onMouseGrid(int event, int x, int y, int foo, void* data) {
    if (event!= CV_EVENT_LBUTTONDOWN) // we just take click event into account
		return;
    Data * D = ((Data*) data);
    
    if (D->counterI2 < 5) { // if we have less than five points, then we had the point to our list and draw a circle on the image
        Mat I = D->I2;
        D->pointsI2.push_back(Point(x,y));
        D->counterI2 ++;
        Point2f m(x, y);
        circle(I, m, 6, Scalar(0, 0, 255), 2); 
        imshow("grid", I);
    }
}


Mat getHomography(VideoCapture camera) {
    Mat I1, I2;
    Data D;
    if (camera.isOpened()) {
        // getting image from live stream. Repetition to avoid buffering
        camera >> I1;
        camera >> I1;
        camera >> I1;
        camera >> I1;
        camera >> I1; // avoiding buffering
        
        // getting image we use to represent the 2D position
        I2 = imread("images/grid.jpg");

        // Data is used to transmit easily several variables through the callback functions
        D.I1 = I1;
        D.I2 = I2;
        D.counterI1 = 0;
        D.counterI2 = 0;
        
        // showing the two images in separate windows
        imshow("image", D.I1);
        imshow("grid", D.I2);
        
        // setting callbacks on clicks for both images
        setMouseCallback("image",onMouseImage, &D);
        setMouseCallback("grid", onMouseGrid, &D);
        
        cout << "Click on 5 points on image and grid, in the same order, then press any key"  << endl;

        while (D.counterI1 < 5 || D.counterI2 < 5) {
            waitKey(0);
        }

        D.H = findHomography(D.pointsI1, D.pointsI2, CV_RANSAC);
    }
    return D.H;
}

