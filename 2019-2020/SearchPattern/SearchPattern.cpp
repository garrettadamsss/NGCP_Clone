#include <cmath>
#include <vector>
#include "SearchPattern.h"
//#include "waypointGenerator.h"
//#include "waypointGenerator_emxutil.h"
using namespace std;
#define TURN_RADIUS 45      //Max turn radius of the plane
#define PI 3.14159265359
//#define TO_RADIANS  PI / 180
#define RADIUS_EARTH    6378037.0


Path::Path(double topLeftLat, double topLeftLong, double alt, double botLeftLat, double botLeftLong, double botRightLat, double botRightLong) {
    Waypoint startPoint = {botLeftLat, botLeftLong, alt};    //Initial point the search starts at
    Waypoint topLeftPoint = {topLeftLat, topLeftLong, alt};       //Top left bound point, used for width
    Waypoint botRightPoint = {botRightLat, botRightLong, alt};     //Bottom right bound point
    Waypoint botLeftPoint = {botLeftLat, botLeftLong, alt};     //Bottom left point used a reference point
    length = gpsDistance(startPoint, botRightPoint);
    width = gpsDistance(startPoint, topLeftPoint);

    verticalPoints = round(width / TURN_RADIUS); //The number of points to be generated in the horizontal direction
    int numPoints = 3 * verticalPoints;
    list.reserve(numPoints);            //List of all waypoints
    count = 0;      //Current number of points already calculated
    list.push_back(startPoint);
    count++;
    x = 0;          //Used for the x location of the plane in an x, y coord system
    y = 0;          //Used for the y location of the plane in an x, y coord system
}
//Converts a change in distance to latitude and longitude points
Waypoint Path::distanceConvert(double deltaX, double deltaY, Waypoint cur) {
    double deltaLat = (deltaY / RADIUS_EARTH);      //Change the change in y to change in latitude
    double deltaLong = deltaX / (RADIUS_EARTH * cos(cur.lat * PI / 180)); //Change the change in x to change in longitude

    //Using the current position, return the new position
    Waypoint newPosition;
    newPosition.lat = cur.lat + (deltaLat * (180 / PI));
    newPosition.lon = cur.lon + deltaLong * ((180 / PI));
    newPosition.alt = cur.alt;
    return newPosition;
}
//Finds the distance in meters between two points
double Path::gpsDistance(Waypoint start, Waypoint end) {
    double startLatRad = start.lat * TO_RADIANS;
    double endLatRad = end.lat * TO_RADIANS;
    double deltaLat = (end.lat - start.lat) * TO_RADIANS;
    double deltaLong = (end.lat - start.lon) * TO_RADIANS;

    double var = sin(deltaLat / 2) * sin(deltaLat / 2)
            + (cos(startLatRad) * cos(endLatRad)) * (sin(deltaLong / 2) * sin(deltaLong / 2));
    double c = 2 * atan2(sqrt(var), sqrt(1 - var));

    return RADIUS_EARTH * c;
}
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: waypointGenerator.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 02-May-2019 23:20:30
//

// Include Files

// Function Definitions

//
// Inputs:
//  vertPoints:   number of vertical points required, found by doing width/TurnRadius
//  topLeft:      top left point of the search area, array [x_TL, y_TL]
//  botRight:     bot right point of the search area, array [x_BL, y_BL]
//  botLeftGPs:   array for lat and long of bot left point
// Arguments    : const double topLeft[2]
//                const double botRight[2]
//                double vertPoints
//                const double botLeftGPS[2]
//                emxArray_real_T *SearchPattern
// Return Type  : void
//
void waypointGenerator(Autopilot_Interface &api, const double topLeftGPS[2], const double botRightGPS[2], int
vertPoints, const double botLeftGPS[2], double alt, double WP_RADIUS)
{
/*(topLeft) X
	    ^
            |(dist1)
            |
            | 
            |   
(botLeft)   X------------->>X  (botRight)
		(dist2)
*/


	//Dist from corner to top left
	mavlink_mission_item_t dist1 = api.distanceVectors(topLeftGPS[0], topLeftGPS[1], botLeftGPS[0], botLeftGPS[1]);
	mavlink_mission_item_t dist2 = api.distanceVectors(botRightGPS[0], botRightGPS[1], botLeftGPS[0], botLeftGPS[1]);


    double left_waypoints[vertPoints][2];
    double mid_waypoints[vertPoints][2];
    double right_waypoints[vertPoints][2];

    double botMid_x;
    double botMid_y;
    double x_distance_iteration;
    double y_distance_iteration;
    int i0;
    int i1;
    int i;
    double d0;
    //int direction;
    double d1;
    //emxInit_real_T(&left_waypoints, 2);
    //emxInit_real_T(&mid_waypoints, 2);
    //emxInit_real_T(&right_waypoints, 2);

    // Function assumes Cartesian plane, then converts to geodetic coordinates
    // In C++ code, use gpsDistance function to find top left and bot right
    // points using bot left as a reference point for deltaLat and deltaLong
    botMid_x = dist2.x / 2.0;
    botMid_y = dist2.y / 2.0;


    x_distance_iteration = dist1.x / vertPoints;
    y_distance_iteration = dist1.y / vertPoints;

/*
    i0 = (int)vertPoints;

    i1 = left_waypoints->size[0] * left_waypoints->size[1];
    left_waypoints->size[0] = i0;
    left_waypoints->size[1] = 2;

    emxEnsureCapacity_real_T(left_waypoints, i1);
    i1 = mid_waypoints->size[0] * mid_waypoints->size[1];
    mid_waypoints->size[0] = i0;
    mid_waypoints->size[1] = 2;

    emxEnsureCapacity_real_T(mid_waypoints, i1);
    i1 = right_waypoints->size[0] * right_waypoints->size[1];
    right_waypoints->size[0] = i0;
    right_waypoints->size[1] = 2;

    emxEnsureCapacity_real_T(right_waypoints, i1);
*/

    for (i = 0; i < vertPoints; i++) {

	left_waypoints[i][0] = x_distance_iteration*i;
	left_waypoints[i][1] = y_distance_iteration*i;
	//printf("Left waypoints %d: %f, %f\n", i, left_waypoints[i][0],left_waypoints[i][1]);
	/*
        d0 = x_distance_iteration * (1.0 + (double)i);
        left_waypoints->data[i] = d0;
        d1 = y_distance_iteration * (1.0 + (double)i);
        left_waypoints->data[i + left_waypoints->size[0]] = d1;
	*/

	mid_waypoints[i][0] = botMid_x + x_distance_iteration * i;
	mid_waypoints[i][1] = botMid_y + y_distance_iteration * i;
//printf("mid waypoints %d: %f, %f\n", i, mid_waypoints[i][0], mid_waypoints[i][1]);
        //mid_waypoints->data[i] = botMid_idx_0 + d0;
        //mid_waypoints->data[i + mid_waypoints->size[0]] = botMid_idx_1 + d1;


	right_waypoints[i][0] = dist2.x + x_distance_iteration * i;
	right_waypoints[i][1] = dist2.y + y_distance_iteration * i;
//printf("right waypoints %d: %f, %f\n", i, right_waypoints[i][0],right_waypoints[i][1]);
        //right_waypoints->data[i] = botRight[0] + d0;
        //right_waypoints->data[i + right_waypoints->size[0]] = botRight[1] + d1;

    }

    int maxWaypoints = 3 * vertPoints;
    double searchPattern[maxWaypoints][2] = {0};

    int direction = 1;
  


    searchPattern[0][0] = 0;
//	 	printf("searchPattern00: %f\n", searchPattern[0][0]);
    searchPattern[0][1] = 0;
    searchPattern[1][0] = botMid_x;
//	 	printf("searchPattern10: %f\n", searchPattern[1][0]);
    searchPattern[1][1] = botMid_y;
    searchPattern[2][0] = dist2.x;  
//	 	printf("searchPattern20: %f\n", searchPattern[2][0]);
    searchPattern[2][1] = dist2.y;
    

/*
    i1 = SearchPattern->size[0] * SearchPattern->size[1];
    direction = (int)x_distance_iteration;
    SearchPattern->size[0] = direction;
    SearchPattern->size[1] = 2;
    emxEnsureCapacity_real_T(SearchPattern, i1);
    direction <<= 1;



    for (i1 = 0; i1 < direction; i1++) {
        SearchPattern->data[i1] = 0.0;
    }

    direction = 1;
    y_distance_iteration = 3.0;
    SearchPattern->data[0] = 0.0;
    SearchPattern->data[SearchPattern->size[0]] = 0.0;
    SearchPattern->data[1] = botMid_idx_0;
    SearchPattern->data[1 + SearchPattern->size[0]] = botMid_idx_1;
    SearchPattern->data[2] = botRight[0];
    SearchPattern->data[2 + SearchPattern->size[0]] = botRight[1];
*/


  int count = 3;
    for (i = 1; i < vertPoints; i++) {
//	printf("run");
        if (direction == 0) {

//		printf("case 1\n");
		searchPattern[count][0] = left_waypoints[i][0];
		searchPattern[count][1] = left_waypoints[i][1];
	 	//printf("search pattern %d: %f, %f\n", count, searchPattern[count][0], searchPattern[count][1]);

		searchPattern[count+1][0] = mid_waypoints[i][0];
		searchPattern[count+1][1] = mid_waypoints[i][1];
	 	//printf("search pattern %d: %f, %f\n", count+1, searchPattern[count+1][0], searchPattern[count+1][1]);

		searchPattern[count+2][0] = right_waypoints[i][0];
		searchPattern[count+2][1] = right_waypoints[i][1];
	 	//printf("search pattern %d: %f, %f\n", count+2, searchPattern[count+2][0], searchPattern[count+2][1]);

	    /*
            SearchPattern->data[(int)(y_distance_iteration + 1.0) - 1] =
                    left_waypoints->data[i];
            SearchPattern->data[((int)(y_distance_iteration + 1.0) +
                                 SearchPattern->size[0]) - 1] = left_waypoints->data[i
                                                                                     + left_waypoints->size[0]];
            SearchPattern->data[(int)(y_distance_iteration + 2.0) - 1] =
                    mid_waypoints->data[i];
            SearchPattern->data[((int)(y_distance_iteration + 2.0) +
                                 SearchPattern->size[0]) - 1] = mid_waypoints->data[i
                                                                                    + mid_waypoints->size[0]];
            SearchPattern->data[(int)(y_distance_iteration + 3.0) - 1] =
                    right_waypoints->data[i];
            SearchPattern->data[((int)(y_distance_iteration + 3.0) +
                                 SearchPattern->size[0]) - 1] = right_waypoints->
                    data[i + right_waypoints->size[0]];
            
	    */


	    direction = 1;
            count += 3.0;

        } else {
//		printf("case 2\n");
		searchPattern[count][0] = right_waypoints[i][0];
		searchPattern[count][1] = right_waypoints[i][1];

		searchPattern[count+1][0] = mid_waypoints[i][0];
		searchPattern[count+1][1] = mid_waypoints[i][1];

		searchPattern[count+2][0] = left_waypoints[i][0];
		searchPattern[count+2][1] = left_waypoints[i][1];

		
/*
            i1 = (int)(y_distance_iteration + 1.0);
            SearchPattern->data[i1 - 1] = right_waypoints->data[i];
            SearchPattern->data[(i1 + SearchPattern->size[0]) - 1] =
                    right_waypoints->data[i + right_waypoints->size[0]];
            i1 = (int)(y_distance_iteration + 2.0);
            SearchPattern->data[i1 - 1] = mid_waypoints->data[i];
            SearchPattern->data[(i1 + SearchPattern->size[0]) - 1] =
                    mid_waypoints->data[i + mid_waypoints->size[0]];
            i1 = (int)(y_distance_iteration + 3.0);
            SearchPattern->data[i1 - 1] = left_waypoints->data[i];
            SearchPattern->data[(i1 + SearchPattern->size[0]) - 1] =
  

            left_waypoints->data[i + left_waypoints->size[0]];
 */          

	    direction = 0;
            count += 3.0;
        }

//printf("count, Search pattern: %d, %f, %f\n", count, searchPattern[count][0], searchPattern[count][1]); 
   }


int j;

for (j = 0; j < count; j++) { 	printf("search pattern %d: %f, %f\n", j, searchPattern[j][0], searchPattern[j][1]);  }


    //emxFree_real_T(&right_waypoints);
    //emxFree_real_T(&mid_waypoints);
    //emxFree_real_T(&left_waypoints);
    //i0 = (int)(x_distance_iteration + 3.0);


    std::vector<mavlink_mission_item_t> waypoints;
    waypoints.push_back(api.create_waypoint(0, 0, 0, 0, 0));


    mavlink_mission_item_t referenceGPS;
    mavlink_mission_item_t position;


    referenceGPS.x = botLeftGPS[0];
    referenceGPS.y = botLeftGPS[1];

	
    for (i = 0; i < maxWaypoints; i++) {

	// CALC LAT AND LON
	position = api.createNewDisplacedWaypoint(searchPattern[i][1], searchPattern[i][0], referenceGPS);
	waypoints.push_back(api.create_waypoint(position.x, position.y, alt, i+1, WP_RADIUS)); //CONVERT INTO MAVLINK READABLE WAYPOINT
	//printf("i+1: %i\n", i);







        //x_distance_iteration = SearchPattern->data[i];

        //  INPUT:
        //  deltaX: desired x distance (in m). This x up in cartesian coordinates
        //  deltaY: desired y distance (in m). This y is West/East direction
        //  pos:    Current position [lat,lon]
        // This function uses the NED frame with +x = North = dLat
        //                                   and +y = East  = dLon
        // km
        //  To meters
        // coordinate offset in Radians
        //SearchPattern->data[i] = botLeftGPS[0] + SearchPattern->data[i +
        //                                                             SearchPattern->size[0]] * 0.001 / 6378.037 * 57.295779513082323;
       // SearchPattern->data[i + SearchPattern->size[0]] = botLeftGPS[1] +
        //                                                  x_distance_iteration * 0.001 / 6378.037 * 57.295779513082323;
    }
	
    uint16_t currentWaypoint = 1;
    api.write_waypoints(waypoints, 1);
    api.setCurrentWaypoint(currentWaypoint);


    return;


}
