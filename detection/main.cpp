/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: alex
 *
 * Created on October 28, 2018, 12:40 PM
 */

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <chrono>

#include "homography.h"
#include "position.h"

using namespace cv;
using namespace std;








VideoCapture initializeCamera() {
    VideoCapture cap(1);
    cap.set(CV_CAP_PROP_BUFFERSIZE, 0);
    
    if(!cap.isOpened()){
        cerr << "No camera detected on this system" << endl;
        return -1;
    }
    return cap;
}

int main( int argc, const char** argv )
{
    VideoCapture camera = initializeCamera();
    Mat H = getHomography(camera);
    Mat image;
    vector<Kobuki> points;
    
    /*
       * THIS CODE ENABLES THE CAMERA AND ACCESS THE VIDEO STREAM AS IMAGES
       */
    while(true){
        camera >> image; // dummy repetitions to avoid buffering...
        camera >> image; // dummy repetitions to avoid buffering...
        camera >> image; // dummy repetitions to avoid buffering...
        camera >> image; // dummy repetitions to avoid buffering...
        camera >> image; // dummy repetitions to avoid buffering...
        if(image.empty()){
            cerr << "Frame invalid and skipped!" << endl;
            continue;
        }
        
        points = getKobukisPositions(image, H);
        for (int i=0 ; i< points.size() ; i++) {
            cout << "Kobuki " << i << " : (" << points[i].x << "," << points[i].y << "), angle=" << points[i].angle << "°" << endl;
        }
        waitKey(5);
        cout << "New image" << endl;
    }
    
//    waitKey(0);
    return 0;
}