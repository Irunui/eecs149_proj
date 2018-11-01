# Detecting the position of Kobukis

This folder contains functions necessary to the detection of Kobukis. There are two major steps:
1) Initializing the system by computing the homography between our 2D-coordinates system and what the webcam sees
2) Computing the Kobukis coordinates from an actual image

You should have OpenCV compiled on your computer and linked to the project before being able to run these files.
You should also have a webcam connected to your computer.

## Initialization and computing homography
Files required:
* homography.cpp
* homography.h
* /maxflow
* /images/grid.jpg

running getHomography() will take a picture with the webcam, then waits for the user to click on 5 points both on the picture and a grid. Once that's done, it computes the homography using these 5 known points, and returns it as a 3x3 Mat.

## Detecting the Kobukis
Files required:
* position.cpp
* position.h
* /maxflow
* /images/grid.jpg

running getKobukisPositions(Mat im, Mat homography) for a given picture (im) and homography (homography) will return the position (x,y) of the Kobukis on the grid.
