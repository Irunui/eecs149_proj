/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "position.h"
#include "homography.h"

/*
 * looking for neighbor pixels to reconstruct Kobukis
 */
int colorNeighbors(int i, int j, int indice, Mat& map, vector<pair<int, int>> &kobuki) {
    map.at<int>(i, j) = indice;
    kobuki.push_back(make_pair(i, j));
    int counter = 1;
    if (i<map.rows - 1 && map.at<int>(i + 1, j) == 1) {
        counter += colorNeighbors(i + 1, j, indice, map, kobuki);
    }
    if (i>0 && map.at<int>(i - 1, j) == 1) {
        counter += colorNeighbors(i - 1, j, indice, map, kobuki);
    }
    if (j<map.cols - 1 && map.at<int>(i, j + 1) == 1) {
        counter += colorNeighbors(i, j + 1, indice, map, kobuki);
    }
    if (j>0 && map.at<int>(i, j - 1) == 1) {
        counter += colorNeighbors(i, j - 1, indice, map, kobuki);
    }
    return counter;
}

/*
 * Given a set of points representing one Kobuki, computes the position of the center of the Kobuki
 */
Point2f coordinates(vector<pair<int, int>> kobuki, int height, int width, Mat& I, Mat H) {
	int max_x = 0;
        int min_x = 100000;
	int max_y = 0;
        int min_y = 1000000;
        int x=0, y =0;
	int nb = 0;

        // This technique uses several coordinates and try to find the center position directly on the image
	for (vector<pair<int, int>>::iterator itr = kobuki.begin(); itr < kobuki.end(); ++itr)
	{
		pair<int, int> pt = *itr;
		if (pt.first > max_x) {
			max_x = pt.first;
		}
                if (pt.first < min_x) {
			min_x = pt.first;
		}
                if (pt.second > max_y) {
			max_y = pt.second;
		}
                if (pt.second < min_y) {
			min_y = pt.second;
		}
	}
        int radius = (max_y - min_y)/2;
	Point m((max_y-radius)*width, (max_x-radius)*height);
	circle(I, m, 6, Scalar(0, 0, 255), 2);
        
        x = (max_x-radius)*height;
        y = (max_y-radius)*width;

	return computePositionOnGrid(y, x, H);
        
//        // This technique uses the bottom position, converts it in grid coordinates and uses a known radius for Kobuki
//        for (vector<pair<int, int>>::iterator itr = kobuki.begin(); itr < kobuki.end(); ++itr)
//	{
//            pair<int, int> pt = *itr;
//            if (pt.first > max_x) {
//                    max_x = pt.first;
//                    y = pt.second;
//            }
//	}
//        
//        Point2f m = computePositionOnGrid(y,max_x,H);
        
}

/*
 * Compute the coordinates of all Kobukis 
 */
vector<Point2f> createGrid(vector<vector<pair<int, int>>> kobukis, int height, int width, Mat im, Mat H) {
	Mat terrain = imread("images/grid.jpg");
        vector<Point2f> points;
	for (vector<vector<pair<int, int>>>::iterator itr = kobukis.begin(); itr < kobukis.end(); ++itr)
	{
		vector<pair<int, int>> kobuki = *itr;
		Point2f pt = coordinates(kobuki, height, width, im, H);
                if (0<= pt.x && pt.x <= terrain.rows && 0 <= pt.y && pt.y<=terrain.rows) {
                    points.push_back(pt);
                    circle(terrain, pt, 2, Scalar(0, 0, 255), 2);
                }
	}
	//imwrite(image+"TERRAIN.png", terrain);
        imshow("grid", terrain);
	imshow("result", im);

	return points;
}

/*
 * From a webcam image and a previously computed homography, returns the positions
 * of every Kobuki detected within the grid limits
 */
vector<Point2f> getKobukisPositions(Mat im, Mat homography)
{    
    // BACKGROUND SUBSTRACTION USING GRAPH CUT
    Mat I2 = im.clone(); // colored image
    Mat I; // black and white image
    cvtColor(I2, I, CV_BGR2GRAY); // transforms to black and white
    int m = I2.rows, n = I2.cols;

    Graph<float, float, float> g(m*n + 1, m*n * 2 + 1); // graph used for graphcut algorithm
    g.add_node(m*n);

    // Parameters
    Vec3b Iext(133, 148, 170), Ikobuki(100, 70, 0);
    float alpha = 1, beta = 0.1;

    // computing gradient
    Mat Grad(m, n, CV_32F);
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            float ix, iy;
            if (i == 0 || i == m - 1 || j == 0 || j == n - 1) {
                ix = 0;
                iy = 0;
            }
            else {
                iy = -float(I.at<uchar>(i - 1, j + 1)) + float(I.at<uchar>(i + 1, j + 1)) - 2 * float(I.at<uchar>(i, j - 1)) + 2 * float(I.at<uchar>(i, j + 1)) - float(I.at<uchar>(i + 1, j - 1)) + float(I.at<uchar>(i + 1, j + 1));
                ix = -float(I.at<uchar>(i - 1, j - 1)) - 2 * float(I.at<uchar>(i - 1, j)) - float(I.at<uchar>(i - 1, j + 1)) + float(I.at<uchar>(i + 1, j - 1)) + 2 * float(I.at<uchar>(i + 1, j)) + float(I.at<uchar>(i + 1, j + 1));
            }
            Grad.at<float>(i, j) = alpha / (1 + beta*ix*ix + beta*iy*iy);
        }
    }
    
    // creating edges	
    for (int i = 0; i < m - 1; i++) {
        for (int j = 0; j < n - 1; j++) {
            if (I2.at<Vec3b>(i, j) != Iext) {// si l'on n'est pas dans le background
                    if (I2.at<Vec3b>(i, j + 1) != Iext) // vers la droite
                            g.add_edge(i*n + j, i*n + j + 1, (Grad.at<float>(i, j) + Grad.at<float>(i, j + 1)) / 2, (Grad.at<float>(i, j) + Grad.at<float>(i, j + 1)) / 2);
                    if (I2.at<Vec3b>(i + 1, j) != Iext)// vers le bas
                            g.add_edge(i*n + j, (i + 1)*n + j, (Grad.at<float>(i, j) + Grad.at<float>(i + 1, j)) / 2, (Grad.at<float>(i, j) + Grad.at<float>(i + 1, j)) / 2);
                    // vers les sources
                    float D0 = norm(I2.at<Vec3b>(i, j), Ikobuki, CV_L2);
                    float D1 = norm(I2.at<Vec3b>(i, j), Iext, CV_L2);
                    g.add_tweights(i*n + j, D0, D1);
            }
        }
    }

    // computing maxflow == the way to separate Kobukis from background
    g.maxflow();

    // Separating background from Kobukis using computed graph maxflow
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++){
            if (I2.at<Vec3b>(i, j) != Iext) {
                if (g.what_segment(i*n + j) == Graph<int, int, int>::SOURCE) {
                    I2.at<Vec3b>(i, j) = Iext;
                }
                else {
                    if (I2.at<Vec3b>(i, j)[1] > 6 * I2.at<Vec3b>(i, j)[0]) {
                            I2.at<Vec3b>(i, j) = Iext;
                    }
                    else {
                            I2.at<Vec3b>(i, j) = Ikobuki;
                    }
                }
            }
        }
    }
    
    // filtering results using basic mean
    I = I2;
    int width = 2;
    int height = 2;
    Mat map = Mat(floor((I.rows - 1) / height), floor((I.cols - 1) / width), CV_32SC1);
    for (int m = 0; m < I.rows - height - 1; m += height) {
        for (int n = 0; n < I.cols - 1 - width; n += width) {
            int counter = 0;
            for (int a = m; a < m + height; a++) {
                for (int b = n; b < n + width; b++) {
                    if (I.at<Vec3b>(m, n) == Ikobuki)
                    {
                            counter++;
                    }
                }
            }
            if (counter >= (width*height / 4)) {
                map.at<int>(floor(m / height), floor(n / width)) = 1;
                for (int a = m; a < m + height; a++) {
                    for (int b = n; b < n + width; b++) {
                            I.at<Vec3b>(a, b) = Vec3b(0, 0, 255);
                    }
                }
            }
            else {
                map.at<int>(floor(m / height), floor(n / width)) = 0;
            }
        }
    }

    // interpreting resulting images to get Kobukis positions
    int indice = 2;
    int nb_kobukis = 0;
    vector<vector<pair<int, int>>> kobukis;
    for (int i = 0; i < (I.rows - height - 1) / height; i++) {
        for (int j = 0; j < (I.cols - width - 1) / width; j++) {
            if (map.at<int>(i, j) == 1) {
                vector<pair<int, int>> kobuki;
                int counter = colorNeighbors(i, j, indice, map, kobuki);
                if (counter > 200) {
                        kobukis.push_back(kobuki);
                        nb_kobukis++;
                }
                indice++;
            }
        }
    }

    return createGrid(kobukis, height, width, im, homography);
}
