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
    } else {
        cout << "clic" << endl;
        Point2f m = computePositionOnGrid(x,y,D->H);

        circle(D->I2, m, 6, Scalar(0, 255, 0), 2);
        imshow("grid", D->I2);
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
    cout << "Retrieve Homography from file ? 1/0" << endl;
    int answer;
    cin >> answer;
    if (answer==1) {
        return retrieveHomographyFromFile();
    }
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
    for (int i =0 ;i < 2 ; i++) {
        cout << "Press a key (multiple times) to stop checking the homography" << endl;
        waitKey(0);
    }
    for (int i=0 ; i < 3 ; i++) {
        for (int j=0 ; j < 3 ; j++) {
            cout << D.H.at<double>(i,j) << endl;
        }
    }
    saveHomographyInFile(D.H);
    return D.H;
}

void saveHomographyInFile(Mat H) {
    ofstream file;
    file.open ("homography.txt");
    for (int i=0 ; i < 3 ; i++) {
        for (int j=0 ; j < 3 ; j++) {
            file << H.at<double>(i,j) << "\n";
        }
    }
    file.close();
}

Mat retrieveHomographyFromFile() {
    Mat H = Mat::zeros(cv::Size(3, 3), CV_64FC1);
    fstream file;
    file.open ("homography.txt", ios::in);
    double temp;
    for (int i=0 ; i < 3 ; i++) {
        for (int j=0 ; j < 3 ; j++) {
            file >> temp;
            H.at<double>(i,j) =  temp;
        }
    }
    for (int i=0 ; i < 3 ; i++) {
        for (int j=0 ; j < 3 ; j++) {
            cout << H.at<double>(i,j) << endl;
        }
    }
    cout << get_angle_transformation(H, 200, 100) << endl;
    waitKey(0);
    file.close();
    return H;
}