
#pragma once

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <mutex>

#include "../Mission_Comms/UAV_Database.h"
#include "Visball.h"
#include "TargetIsolator.h"
#include "GPSLocation.h"
#include "ImageConverter.h"
#include "ThermalFailsafe.h"


#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/features2d.hpp" 
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/video/background_segm.hpp>

using namespace std;
using namespace cv;

class ThermalFailsafe;

class ISR{
private:
    grip::Visball *visball;
    grip::TargetIsolator *bullseye;
    ThermalFailsafe *thermal_detector;
    cv::VideoCapture camera;
    ImageConverter img;
    std::ofstream log_file;

    bool DEBUG_ENABLED = false;

    GPSLocation ballGPS;
    GPSLocation targetGPS;
    GPSLocation thermalGPS;

    UAV_Database uav_db;

    // Vectors that hold list of contours, each contour is a list of points
    vector<vector<Point> > ballPoints;
    vector<vector<Point> > bullseyePoints;
    vector<Point> thermal_flag_centers;

    double alt;
    double lat;
    double lon;
    double yaw;

    bool ready = true;

    void log(String message);
    double grab_altitude();
    void array3d(Mat small, int array[][100][100]);
    void reCreate(int array[][100][100]);
    void process_mats(Mat img, vector<vector<Point> > *ballPoints, vector<vector<Point> > *bullseyePoints);
    void process_contour(vector<vector<Point> > hullsOutput, cv::Point *pt, int *radius);
    void send_to_db(cv::Point point, double radius, GPSLocation gps, cv::Mat imgOriginal, int db_index);
    void init();
public:
    ISR();
    int run(bool visible_search);
    void end();
    void start(bool *visible_search, mutex *lock, double *gps_info, double *yaw, int *mission_status);
};
