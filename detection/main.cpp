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

#include "mqtt/async_client.h"

#include "homography.h"
#include "position.h"
#include "mqtt.h"

using namespace cv;
using namespace std;




const float REALITY_COEF = 1.0/370; //to convert our measure to meters
const float DIST_TO_CENTER = 400;



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
    
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);
    
                mqtt::const_message_ptr msg;
    mqtt::async_client cli(SERVER_ADDRESS, CLIENT_ID);

    try {
        cout << "Connecting to the MQTT server..." << flush;
        cli.connect(connOpts)->wait();
        cli.start_consuming();
        cli.subscribe("change", QOS)->wait();
        cli.subscribe(TOPIC2, QOS)->wait();
        cout << "OK" << endl;
        mqtt::message_ptr pubmsg;
        VideoCapture camera = initializeCamera();
        Mat H = getHomography(camera);
        Mat image;
        vector<Kobuki> points, pastPoints;
        
        // initializes feature detection
       
    
        /*
           * THIS CODE ENABLES THE CAMERA AND ACCESS THE VIDEO STREAM AS IMAGES
           */
        while(true){
            try {
                cout << "0";
                camera >> image; // dummy repetitions to avoid buffering...
                camera >> image; // dummy repetitions to avoid buffering...
                camera >> image; // dummy repetitions to avoid buffering...
                camera >> image; // dummy repetitions to avoid buffering...
                camera >> image; // dummy repetitions to avoid buffering...
                cout << "0b";
                if(image.empty()){
                    cerr << "Frame invalid and skipped!" << endl;
                    continue;
                }
                cout << "0c";

                points = getKobukisPositions(image, H);
                cout << "Number of kobukis : " << points.size() << endl;
                points = identifyKobukis(pastPoints, points);
                if (pastPoints.size()==points.size()) {
                    for (int i=0 ; i< points.size() ; i++) {
                        cout << "Kobuki " << points[i].id << " : (" << points[i].x << "," << points[i].y << "), angle=" << points[i].angle << "°" << endl;
                        if (true || ((points[i].x-pastPoints[i].x)*(points[i].x-pastPoints[i].x)+(points[i].y-pastPoints[i].y)*(points[i].y-pastPoints[i].y)+(points[i].angle-pastPoints[i].angle)*(points[i].angle-pastPoints[i].angle) > 15)) {
                            cout << "Publishing " << endl;
                            pubmsg = mqtt::make_message(TOPIC2, std::to_string(points[i].id)+";"+std::to_string((points[i].x-DIST_TO_CENTER)*REALITY_COEF)+";"+std::to_string(-(points[i].y-DIST_TO_CENTER)*REALITY_COEF)+";"+std::to_string(points[i].angle)+";"+std::to_string(points[i].timestamp));
                            pubmsg->set_qos(QOS);
                            cli.publish(pubmsg)->wait_for(TIMEOUT);
                        }
                    }
                }
                pastPoints = points;
                if (cli.try_consume_message(&msg)) {
                    string s = msg.get()->get_payload_str();
                    if (s.size() == 2) {
                        int a = ((int) s[0]) - ((int) '0');
                        int b = ((int) s[1]) - ((int) '0');
                        cout << a << " " << b << endl;
                        
                        for (int i=0; i<points.size() ; i++) {
                            if (points[i].id ==a) {
                                points[i].id = b;
                            } else if (points[i].id==b) {
                                points[i].id=a;
                            }
                        }
                        waitKey(0);
                    }
                    
                }
                waitKey(5);
                cout << "New image" << endl;
            } catch (Exception e) {
                cout << "Exception ICI balec" << endl;
                sleep(2);
            }
            
        }
        
        cout << "\nShutting down and disconnecting from the MQTT server..." << flush;
        cli.unsubscribe(TOPIC)->wait();
        cli.stop_consuming();
        cli.disconnect()->wait();
        cout << "OK" << endl;
    }
    catch (const mqtt::exception& exc) { 
	}
    
//    waitKey(0);
    return 0;
}