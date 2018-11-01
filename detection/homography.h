/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   homography.h
 * Author: alex
 *
 * Created on October 31, 2018, 2:56 PM
 */

#ifndef HOMOGRAPHY_H
#define HOMOGRAPHY_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unistd.h>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

Point2f computePositionOnGrid(float x, float y, Mat H);
Mat getHomography(VideoCapture camera);

#endif /* HOMOGRAPHY_H */

