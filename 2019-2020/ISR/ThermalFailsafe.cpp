//
// Created by zach on 2/22/19.
//

#include "ThermalFailsafe.h"

using namespace std;
using namespace cv;

ThermalFailsafe::ThermalFailsafe(){
    thermal_detector = new grip::IRHuman();
}

// Process image to find all contour midpoints, scaled to correct resolution
// Pass contour midpoint list into db insertion function
// Return the number of contours processed
//TODO convert to string for verify
vector<Point> ThermalFailsafe::process_img(Mat img, double altitude){
    thermal_detector->Process(img, altitude);
    vector<vector<Point> > detected_contours = thermal_detector->GetConvexHullsOutput();
    vector<Point> contour_midpoints;

    for(int i = 0; i < detected_contours.size(); i++){
        Point midpoint;
        for(int point = 0; point < detected_contours[i].size(); point++){
            midpoint.x += detected_contours[i][point].x;
            midpoint.y += detected_contours[i][point].y;
        }
        midpoint.x /= detected_contours[i].size();
        midpoint.y /= detected_contours[i].size();

        contour_midpoints.push_back(midpoint);
        
    }
    queue_in_db(contour_midpoints);
    return contour_midpoints;
}

void ThermalFailsafe::queue_in_db(std::vector<cv::Point>){
    //TODO loop through and insert midpoints into database (or through whatever comm protocol)
}
