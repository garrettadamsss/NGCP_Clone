/**
 *
 * NGCP UAV - Mission Communication Program
 * Author: Tristan Cady, Student ECE Department
 *
 **/


#ifndef UAV_Database_H
#define UAV_Database_H

#include <string>
#include "soci/soci.h"
#include "soci/mysql/soci-mysql.h"
#include <iostream>
#include <istream>
#include <ostream>
#include <fstream>
#include <string>
#include <exception>

using namespace soci;

class UAV_Database {
private:
    session sql;
public:
    UAV_Database();

    void Connect(std::string, std::string, std::string);

    void InsertGPS_LOCAL(double, double, double);

    void InsertHEADING_LOCAL(double, double, double);

    void InsertWAYPOINT_LOCAL(double, double, double);

    void InsertTARGET_LOCAL(double, double, double, int);

    void InsertGPS_ALT(double, double, double);

    void InsertHEADING_ALT(double, double, double);

    void InsertWAYPOINT_ALT(double, double, double);

    void PullGPS_LOCAL(double *, int);

    void PullHEADING_LOCAL(double *, int);

    void PullWAYPOINT_LOCAL(double *, int);

    void PullTARGET_LOCAL(double *, int *);

    void PullGPS_ALT(double *, int);

    void PullHEADING_ALT(double *, int);

    void PullWAYPOINT_ALT(double *, int);

    void PullLatestGPS_LOCAL(double *);

    void PullLatestWAYPOINT_LOCAL(double *);

    void PullLatestHEADING_LOCAL(double *);

    void PullLatestGPS_ALT(double *);

    void PullLatestWAYPOINT_ALT(double *);

    void PullLatestHEADING_ALT(double *);

    void DropGPS_LOCAL();

    void DropHEADING_LOCAL();

    void DropWAYPOINT_LOCAL();

    void DropTARGET_LOCAL();

    void DropGPS_ALT();

    void DropHEADING_ALT();

    void DropWAYPOINT_ALT();

    void Query(std::string);

    void DatabaseDisconnect();

    void InsertPixelTARGET(int, int);

    void PullPixelTARGET(int *xy);

    void SetMissionStatus(int);

    int CheckMissionStatus();

    void SetTargetStatus(int);

    int CheckTargetStatus();

    int flightdatamigration();

    void InsertImageString_S(std::string);

    std::string PullImageString_S(int);

    int ImageStringNum_S();

    void InsertImageString_R(std::string, int);

    std::string PullImageString_R(int, int);

    int ImageStringNum_R();

    void ClearImageString_S();

    void ClearImageString_R();

    void InsertMission(std::string);

    std::string PullMission();
};
#endif
