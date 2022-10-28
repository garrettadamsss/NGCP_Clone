#ifndef BALLLOCATION_H
#define BALLLOCATION_H


class GPSLocation {
private:
    double altitude;
    double horizontalFOV;
    double verticalFOV;
    double pi;
    double heading;
    double currentLat;
    double currentLong;
    double LatOffsetGPS;
    double LongOffsetGPS;
    double curLatEstimate;
    double curLongEstimate;
    int num;
    int xBall;
    int yBall;
    int imageWidth;
    int imageHeight;

public:
    GPSLocation();
    double toRadians(double deg);
    double modDouble(double num, int mod);
    double longOffset(double meters);
    double latOffset(double meters);
    double getLatGPS();
    double getLongGPS();
    double trueAngle(double heading, double tempAngle);
    void averageGPSpoints(double newLat, double newLong);
    void addBL(int tempXBall, int tempYBall, double tempHeight, double tempHeading, double tempLat, double tempLong);
};

#endif
