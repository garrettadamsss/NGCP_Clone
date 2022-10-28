/*
 * SearchPattern.h
 *
 *  Created on: May 4, 2018
 *      Author: ejkrapil
 */

#ifndef SEARCHPATTERN_H
#define SEARCHPATTERN_H
#include "../autopilot_interface.h"
#include <vector>
#include <iostream>


void waypointGenerator(Autopilot_Interface &api, const double topLeftGPS[2], const double botRightGPS[2], int
vertPoints, const double botLeftGPS[2], double alt, double WP_RADIUS);

class Path
{
private:
    std::vector<Waypoint> list;
    double deltaX;          //Change in the x distance to the next point (meters)
    double deltaY;          //Change in the y distance to the next point (meters)
    double x;               //X point of the next point
    double y;               //Y point of the next point
    double length;          //length of the field (meters)
    double width;           //width of the field (meters)
    int count;              //Index of the list of points
    int verticalPoints;     //Number of points in the vertical pass
    Waypoint distanceConvert(double deltaX, double deltaY, Waypoint cur);
    double gpsDistance(Waypoint, Waypoint);
public:
    Path(double topLeftLat, double topLeftLong, double alt, double botLeftLat, double botLeftLong, double botRightLat, double botRightLong);
    std::vector<Waypoint> calculations();
};


#endif /* SEARCHPATTERN_H_ */
