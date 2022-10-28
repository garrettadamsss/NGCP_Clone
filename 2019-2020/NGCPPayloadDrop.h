//
// Created by root on 4/21/18.
//

#ifndef PAYLOAD_NGCPPAYLOADDROP_H
#define PAYLOAD_NGCPPAYLOADDROP_H

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <math.h>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <chrono>

#include <vector>

using namespace std;

#include "Mavlink/common/mavlink.h"

#include "serial_port.h"
#include "Mission_Comms/UAV_Database.h"
#include "NGCPPayloadDrop.h"

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
//sudo apt-get install libboost-all-dev

#include "autopilot_interface.h"

class NGCPPayloadDrop{
private:
    double vel;
    double alt;
    double w1;
public:
    NGCPPayloadDrop(double a, double v, double w);
    double xvalue();
    double toRadians(float deg);
    double distanceFromBall(float x1, float y1, float curLat, float curLong);
    double timeToDrop(double *target, double *curLocation, double heading, double speed);
};

// ------------------------------------------------------------------------------
//   Payload Object
// ------------------------------------------------------------------------------

class Payload_Drop{ //Made by Alex Winger and Zach Cheben
private:
    double time;			//The time it takes to get to the point we need to release the payload
    double PLong;			//The longitude of the plane
    double PLat;			//The latitude of the plane
    double velocity;		//The velocity of the plane
    double altitude;		//The alt of the plane
    double targetDistance;	//The distance from the plane to the target
    double dropDistance;	//The distance the payload drops once it is dropped
    double distance;		//The distance from the plane to the point the payload should be dropped

public:

    double TLat;			//The latitude of the target
    double TLong;			//The longitude of the target

    Payload_Drop(double tlat, double tlong, double plat, double plong, double a);

    void setGPSinfo(double plat, double plong, double a);		//Sets the GPS information of the plane
    void setVelocity(int vx, int vy, int vz);

    double timeToDrop(); //Returns the time it will take to reach the drop point

    bool near_target(Autopilot_Interface &api, const int &radius); //Rudimentary code to drop payload. Use as last resort


    /*
     * Helper Functions
     */
    vector<Waypoint> payload_waypoints(const int &first_distance, const int &second_distance,
                                       const double &angle); //Returns 3 waypoints
    Waypoint meterDisplacement(const double & deltaX, const double & deltaY, const Waypoint & b);

    double gpsDistance(const double &target_lat, const double &target_long, const double &current_lat,
                       const double &current_long); //Function to convert latitude and longitude into a deistance (Measured in meters)

};
#endif //PAYLOAD_NGCPPAYLOADDROP_H
