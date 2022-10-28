
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <stdio.h>
#include "GPSLocation.h"
using namespace std;

GPSLocation :: GPSLocation() {
    pi = 3.1415926535897;
    curLatEstimate = 0;
    curLongEstimate = 0;
    num = 0;
}

double GPSLocation :: toRadians(double deg) {
    return pi * deg / 180.0;
}

double GPSLocation :: modDouble(double num, int mod) {
    double right = num - (double) ((int) num);
    int tempMod = ((int) num) % mod;
    return (double) tempMod + right;
}

double GPSLocation :: longOffset(double meters) {
    double oneLong = toRadians(6378137.0) * cos(toRadians(currentLat));
    return (double) meters / oneLong;
}

double GPSLocation :: latOffset(double meters) {
    double oneLat = 111132.954 - 559.822 * cos(2 * toRadians(currentLat))
            + 1.175 * cos(4 * toRadians(currentLat));
    return (double) meters / oneLat;
}

double GPSLocation :: getLatGPS() {
    return curLatEstimate;
}

double GPSLocation :: getLongGPS() {
    return curLongEstimate;
}

/*
 * @param heading is the heading of the plane in DEGREES
 * @param tempAngle is the heading of the ball relative to the center of the image
 *  in RADIANS
 * TODO: this is converting NED frame to cartesian. The better solution is to put leave the code in NED frame
 */
double GPSLocation :: trueAngle(double heading, double tempAngle) { //returns the angle of the ball in respect to true east in radians
    //cout << tempAngle << endl;
    double hm90 = modDouble(heading, 90);
    if (heading >= 270)
        heading = 180.0 - hm90; //need to convert heading to polar angle
    else if (heading >= 180)
        heading = 270.0 - hm90;
    else if (heading >= 90)
        heading = 360.0 - hm90;
    else
        heading = 90.0 - hm90;
    heading = modDouble(heading, 360);
    heading = toRadians(heading);
    return heading + tempAngle - pi / 2; //add the two angles and subtract 90 degrees

}

void GPSLocation :: averageGPSpoints(double newLat, double newLong) {
    curLatEstimate = curLatEstimate * num + newLat;
    curLongEstimate = curLongEstimate * num + newLong;
    num++;
    curLatEstimate /= num;
    curLongEstimate /= num;
}

void GPSLocation :: addBL(int tempXBall, int tempYBall, double tempHeight, double tempHeading, double tempLat,
        double tempLong) {

    xBall = tempXBall;
    yBall = tempYBall;
    altitude = tempHeight;
    heading = tempHeading;

//  horizontalFOV = 0.5236; //30 degrees, David's estimate
    /* 1.48353 rads = 85 degrees, calculated from camera focal length of 3.8 mm
     * with image format of 1/2", 16/9 aspect ratio using the following website's conversion
     * https://www.usa.canon.com/internet/portal/us/home/explore/learning-center/electronic-range-calculators/bctv-range-angleview
     */
    //horizontalFOV = 1.16; //last year's value
    horizontalFOV = 0.994838; //current value, assuming spec sheet value of 57 degrees from https://www.flir.com/globalassets/imported-assets/document/flir-duo-r-datasheet-en.pdf

    imageWidth = 1920;
    imageHeight = 1080;
    currentLat = tempLat;
    currentLong = tempLong;
    //curLat = 34.0589;//latitude;
    //curLong = 117.8248;//longitude

    int center[] = { (imageWidth / 2), (imageHeight / 2) };
    int deltaX = xBall - *center; // deltaX/deltaY are the x/y offset of the...
    int deltaY = yBall - *(center + 1); //... ball from the center of the image
    double frameWidth = 2 * tan(horizontalFOV / 2) * altitude; //width of field below in meters
    double metersPerPixel = frameWidth / imageWidth;
    double pixelBallDistance = sqrt((double) (deltaX * deltaX + deltaY * deltaY)); //pythagorean theorem
    double distanceFromBall = metersPerPixel * pixelBallDistance; //convert pixel distance to meters
    double angle = trueAngle(heading, atan2((double) -deltaY, (double) deltaX)); //gets polar angle
    //cout << angle << endl;
    LatOffsetGPS = latOffset(sin(angle) * distanceFromBall); //converts gets difference in GPS coordinates
    LongOffsetGPS = longOffset(cos(angle) * distanceFromBall); //ultimately gets added to current GPS coordinate
    //cout << "Distance:" << distanceFromBall << endl << "Angle:" << angle << endl;
    //cout << "Latitude Offset:" << LatOffsetGPS << endl << "Longitude Offset:" << LongOffsetGPS
            //<< endl;
    averageGPSpoints(LatOffsetGPS + currentLat, LongOffsetGPS + currentLong);
}

