/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   position.h
 * Author: alex
 *
 * Created on October 31, 2018, 3:38 PM
 */

#ifndef POSITION_H
#define POSITION_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unistd.h>
#include <iostream>
#include <fstream>


#include "maxflow/graph.h"

using namespace cv;
using namespace std;

const float KOBUKI_REAL_RADIUS = 170.0; // Kobuki real size in mm
const float GRID_REAL_HEIGHT = 300*5; // grid real width in mm

struct Kobuki {
    float x, y, angle;
    int timestamp;
};

const Mat kobuki_reference_im = imread("images/top 1 (copy).jpg", CV_LOAD_IMAGE_GRAYSCALE);

vector<Kobuki> getKobukisPositions(Mat im, Mat homography);

#endif /* POSITION_H */

