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
    int id;
    float x, y, angle;
    int timestamp;
};

const int NB_REFERENCE_IMAGES = 4;
const Mat kobuki_reference_im1 = imread("images/kobuki_ref_1_bis.jpg", CV_LOAD_IMAGE_GRAYSCALE);
const Mat kobuki_reference_im2 = imread("images/kobuki_ref_2.jpg", CV_LOAD_IMAGE_GRAYSCALE);
const Mat kobuki_reference_im3 = imread("images/kobuki_ref_3bis.jpg", CV_LOAD_IMAGE_GRAYSCALE);
const Mat kobuki_reference_im4 = imread("images/kobuki_ref_4.jpg", CV_LOAD_IMAGE_GRAYSCALE);
const Mat kobuki_reference_im5 = imread("images/kobuki_ref_5.jpg", CV_LOAD_IMAGE_GRAYSCALE);
const Mat kobuki_reference_im6 = imread("images/kobuki_ref_6.jpg", CV_LOAD_IMAGE_GRAYSCALE);
const Mat kobuki_reference_im7 = imread("images/kobuki_ref_7.jpg", CV_LOAD_IMAGE_GRAYSCALE);
//vector<KeyPoint> kp1;
//Mat desc1;
//Ptr<AKAZE> D = AKAZE::create();

vector<Kobuki> getKobukisPositions(Mat im, Mat homography);
vector<Kobuki> identifyKobukis(vector<Kobuki> previousKobukis, vector<Kobuki> newKobukis);
float get_angle_transformation(Mat H, float x, float y);

#endif /* POSITION_H */

