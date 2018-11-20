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
 * GET Homography between kobuki reference image and kobuki detected
 */
Mat get_kobuki_homography(Mat kobuki_im) {
    Mat I1 = kobuki_reference_im;
    Ptr<AKAZE> D = AKAZE::create();
    vector<KeyPoint> kp1, kp2;
    Mat desc1, desc2;
    D->detectAndCompute(I1, noArray(), kp1, desc1);
    D->detectAndCompute(kobuki_im, noArray(), kp2, desc2);
    
    if (kp1.size()>0 && kp2.size()>0) {
        Mat J;

        // find and filter matches
        BFMatcher M(NORM_HAMMING);
        vector<vector<DMatch>> nn_matches;
        vector <DMatch> good_matches;
        vector<KeyPoint> kp_1, kp_2;
        int j = 0;
        M.knnMatch(desc1, desc2, nn_matches, 2);
        for (int i = 0; i < nn_matches.size() && kp2.size()!=0; i++) {
            cout << i << endl;
                DMatch first = nn_matches[i][0];
                float dist1 = nn_matches[i][0].distance;
                float dist2 = nn_matches[i][1].distance;
                if (dist1 < 0.6 * dist2) {
                        kp_1.push_back(kp1[first.queryIdx]);
                        kp_2.push_back(kp2[first.trainIdx]);
                        good_matches.push_back(DMatch(j,j,0));
                        j++;
                }
        }

//        // drawMatches ...
//        Mat J2;
//        drawMatches(I1, kp_1, kobuki_im, kp_2, good_matches, J2);
//        namedWindow("J", 1);
//        waitKey(0);

        // find homography
        if (kp_1.size()< 4) {
            return Mat();
        }
        vector<Point2f> KP1, KP2;
        for (int i = 0; i < kp_1.size(); i++) {
                KP1.push_back(kp_1[i].pt);
                KP2.push_back(kp_2[i].pt); 
        }
        Mat H = findHomography(KP1, KP2, CV_RANSAC);

        // find inliers and new homography
        vector<Point2f> inliers1, inliers2;
        for (int i = 0; i < kp_1.size(); i++) {
                Mat pt = Mat::ones(3, 1, CV_64F);
                pt.at<double>(0) = KP1[i].x;
                pt.at<double>(1) = KP1[i].y;
                pt = H * pt;
                pt /= pt.at<double>(2);
                float d = sqrt(pow(pt.at<double>(0) - KP2[i].x, 2) + pow(pt.at<double>(1) - KP2[i].y, 2));
                if (d < 1) {
                        inliers1.push_back(KP1[i]);
                        inliers2.push_back(KP2[i]);
                }
        }
        
        if (inliers1.size() < 4) {
            return Mat();
        }
        H = findHomography(inliers1, inliers2, CV_RANSAC);
        return H;
    } else {
        return Mat();
    }
}

/*
 * GET Kobuki center position from reference homography
 */

Point2f get_kobuki_center(Mat H) {
    Mat I1 = kobuki_reference_im;
    Point2f center1 = Point2f(I1.rows/2, I1.cols/2);
    return computePositionOnGrid(center1.x, center1.y, H);
}

/*
 * GET Kobuki angle from reference homography
 */
float get_kobuki_angle(Mat H) {
    Mat I1 = kobuki_reference_im;
    Point2f center1 = Point2f(I1.rows/2, I1.cols/2);
    Point2f top1 = Point2f(I1.rows, I1.cols/2);
    Point2f center2 = computePositionOnGrid(center1.x, center1.y, H);
    Point2f top2 = computePositionOnGrid(top1.x, top1.y, H);

    const float PI = 3.1415;
    float tangente = (top2.y-center2.y)/(top2.x-center2.x);
    float cosinus = (top2.x-center2.x);
    float angle = atan(tangente); // angle between -PI/2 and PI/2
    cout << angle << endl;
    if (cosinus<0) {
        angle += PI;// angle between -PI/2 and 3PI/2
    }
    if (angle < 0) {
        angle += 2*PI; //angle between 0 and 2PI
    }
    angle *= 180/PI; // angle between 0 and 360°
    cout << angle << "°" << endl;
    return angle;
}


/*
 * Given a set of points representing one Kobuki, computes the position of the center of the Kobuki
 */
Kobuki coordinates(vector<pair<int, int>> kobuki, int height, int width, Mat& I, Mat H) {
    const int FILTER = 10;
    int max_x = 0;
    int min_x = 100000;
    int max_y = 0;
    int min_y = 1000000;
    int x=0, y =0;
    int nb = 0;
    int x_at_bottom[FILTER], y_at_bottom[FILTER];
    
    for (int i=0 ; i<FILTER ; i++) {
        x_at_bottom[i] = 0;
    }

        // This technique uses several coordinates and try to find the center position directly on the image
//	for (vector<pair<int, int>>::iterator itr = kobuki.begin(); itr < kobuki.end(); ++itr)
//	{
//		pair<int, int> pt = *itr;
//		if (pt.first > max_x) {
//			max_x = pt.first;
//		}
//                if (pt.first < min_x) {
//			min_x = pt.first;
//		}
//                if (pt.second > max_y) {
//			max_y = pt.second;
//		}
//                if (pt.second < min_y) {
//			min_y = pt.second;
//		}
//	}
//        int radius = (max_y - min_y)/2;
//	Point m((max_y-radius)*width, (max_x-radius)*height);
//	circle(I, m, 6, Scalar(0, 0, 255), 2);
//        
//        x = (max_x-radius)*height;
//        y = (max_y-radius)*width;
//
//	return computePositionOnGrid(y, x, H);
        
        // This technique uses the bottom position, converts it in grid coordinates and uses a known radius for Kobuki
    for (vector<pair<int, int>>::iterator itr = kobuki.begin(); itr < kobuki.end(); ++itr)
    {
        pair<int, int> pt = *itr;
        for(int i=0 ; i<FILTER; i++) {
            if (x_at_bottom[i]<pt.first) {
                for (int j=FILTER-2; j>=i ; j--) {
                    x_at_bottom[j+1] = x_at_bottom[j];
                    y_at_bottom[j+1] = y_at_bottom[j];
                }
                x_at_bottom[i] = pt.first;
                y_at_bottom[i] = pt.second;
                break;
            }
        }
        if (pt.first < min_x) {
            min_x = pt.first;
        }
        if (pt.first > max_x) {
                max_x = pt.first;
        }
        if (pt.second > max_y) {
            max_y = pt.second;
        }
        if (pt.second < min_y) {
            min_y = pt.second;
        }
    }
        
    x=0;
    y=0;
    int count = 0;
    for (int j=0; j<FILTER; j++) {
        if (x_at_bottom[j]!=0) {
            x += x_at_bottom[j];
            y += y_at_bottom[j];
            count ++;
        } else {
            break;
        }
    }
    x /= count;
    y /= count;
    
    

    
    Mat kobukiIm;
    float WINDOW = 1.2;
    getRectSubPix(I, Size((max_y-min_y)*WINDOW*width, (max_x-min_x)*WINDOW*height), Point2f((max_y+min_y)/2*width,(max_x+min_x)/2*height), kobukiIm);
    Mat H_kobuki = get_kobuki_homography(kobukiIm); 
    if (H_kobuki.rows > 1) {
        Point2f center_rect = get_kobuki_center(H_kobuki);
        float x_center = center_rect.x+(max_y+min_y)/2*width - (max_y-min_y)*WINDOW*width/2;
        float y_center = center_rect.y + (max_x+min_x)/2*height - (max_x-min_x)*WINDOW*height/2;
        circle(I, Point2f(x_center,y_center), 6, Scalar(0, 0, 255), 2);
        float angle = get_kobuki_angle(H_kobuki);
        imshow("kobuki", kobukiIm);
        imshow("i", I);
        waitKey(0);
        
    
        Point2f m = computePositionOnGrid(x_center,y_center,H);

        Mat terrain = imread("images/grid.jpg");
        cout << m.x << " " << KOBUKI_REAL_RADIUS/GRID_REAL_HEIGHT * terrain.rows << endl;
        m.y -= KOBUKI_REAL_RADIUS/GRID_REAL_HEIGHT * terrain.rows;
        Kobuki res = Kobuki();
        res.x = m.x;
        res.y = m.y;
        res.angle = angle;
        res.timestamp = 0;
        return res; 
    } else {
        return Kobuki();
    }
}

/*
 * Compute the coordinates of all Kobukis 
 */
vector<Kobuki> createGrid(vector<vector<pair<int, int>>> kobukis, int height, int width, Mat im, Mat H) {
	Mat terrain = imread("images/grid.jpg");
        vector<Kobuki> points;
	for (vector<vector<pair<int, int>>>::iterator itr = kobukis.begin(); itr < kobukis.end(); ++itr)
	{
		vector<pair<int, int>> kobuki = *itr;
		Kobuki pt = coordinates(kobuki, height, width, im, H);
                if (0< pt.x && pt.x <= terrain.rows && 0 < pt.y && pt.y<=terrain.rows) {
                    points.push_back(pt);
                    circle(terrain, Point2f(pt.x, pt.y), 2, Scalar(0, 0, 255), 2);
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
vector<Kobuki> getKobukisPositions(Mat im, Mat homography)
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