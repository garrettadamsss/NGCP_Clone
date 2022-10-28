/**
 *
 * NGCP UAV - Mission Communication Program
 * Author: Tristan Cady, Student ECE Department
 *
 **/


#include "UAV_Database.h"
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
using namespace std;

UAV_Database::UAV_Database() = default;

void UAV_Database::Connect(string dbname,string username,string password)
{
    try
    {
    	string sqlconinfo = "db=" + dbname + " " + "user=" + username + " " + "password=" + password;
    	cout << sqlconinfo << endl; //For Debug
    	sql.open(mysql, sqlconinfo);
    	sql << "CREATE TABLE IF NOT EXISTS gps_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS gps_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS heading_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, yaw double NOT NULL, pitch double NOT NULL, roll double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS heading_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, yaw double NOT NULL, pitch double NOT NULL, roll double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS waypoint_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS waypoint_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS target_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, target_id int NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP)";
    	sql << "CREATE TABLE IF NOT EXISTS target_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP)";
    	sql << "CREATE TABLE IF NOT EXISTS pixeltarget (id int NOT NULL PRIMARY KEY, x INT NOT NULL, y INT NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS missionstatus (id int NOT NULL PRIMARY KEY, status BOOL NOT NULL)";
        sql << "CREATE TABLE IF NOT EXISTS flight_id (id int NOT NULL PRIMARY KEY, flight_id INT NOT NULL)";
        sql << "CREATE TABLE IF NOT EXISTS ach_gps_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, flight_id int NOT NULL ,altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP NULL DEFAULT NULL)";
        sql << "CREATE TABLE IF NOT EXISTS ach_gps_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, flight_id int NOT NULL , altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP NULL DEFAULT NULL)";
        sql << "CREATE TABLE IF NOT EXISTS ach_heading_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, flight_id int NOT NULL , yaw double NOT NULL, pitch double NOT NULL, roll double NOT NULL, time TIMESTAMP NULL DEFAULT NULL)";
        sql << "CREATE TABLE IF NOT EXISTS ach_heading_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, flight_id int NOT NULL , yaw double NOT NULL, pitch double NOT NULL, roll double NOT NULL, time TIMESTAMP NULL DEFAULT NULL)";
        sql << "CREATE TABLE IF NOT EXISTS ach_waypoint_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, flight_id int NOT NULL , altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP NULL DEFAULT NULL)";
        sql << "CREATE TABLE IF NOT EXISTS ach_waypoint_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, flight_id int NOT NULL , altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP NULL DEFAULT NULL)";
        sql << "CREATE TABLE IF NOT EXISTS ach_target_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, flight_id int NOT NULL , altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP NULL DEFAULT NULL)";
        sql << "CREATE TABLE IF NOT EXISTS ach_target_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, flight_id int NOT NULL , altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP NULL DEFAULT NULL)";
        sql << "CREATE TABLE IF NOT EXISTS image_strings_s (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, token text NOT NULL, target_id int NOT NULL)";
		sql << "CREATE TABLE IF NOT EXISTS image_strings_r (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, token text NOT NULL, target_id int NOT NULL)";
        sql << "CREATE TABLE IF NOT EXISTS targetstatus (id int NOT NULL PRIMARY KEY, status INT NOT NULL)";
		sql << "CREATE TABLE IF NOT EXISTS missionjson (id int NOT NULL PRIMARY KEY, msg text NOT NULL)";
		sql << "SET GLOBAL max_allowed_packet=1073741824;";
    	sql << "SET GLOBAL net_read_timeout = 200 ;";
    	sql << "SET GLOBAL connect_timeout = 200 ;";
    	sql << "SET GLOBAL max_connections = 2000 ;";
    	//sql.set_log_stream(ostream);
    }
    catch (exception const &e)
    {
        cerr << "Error: " << e.what() << '\n';
    }
}

void UAV_Database::InsertGPS_LOCAL(double altitude, double latitude, double longitude)
{
	sql << "CREATE TABLE IF NOT EXISTS gps_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
	statement sqlgpsinfo = (sql.prepare << "INSERT INTO gps_local (altitude,latitude,longitude,pulled) VALUES (:f,:f,:f,:b)", use(altitude),use(latitude),use(longitude),use(0));
	sqlgpsinfo.execute(true);
}
void UAV_Database::InsertGPS_ALT(double altitude, double latitude, double longitude)
{
	sql << "CREATE TABLE IF NOT EXISTS gps_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
	statement sqlgpsinfo = (sql.prepare << "INSERT INTO gps_alt (altitude,latitude,longitude,pulled) VALUES (:f,:f,:f,:b)", use(altitude),use(latitude),use(longitude),use(0));
	sqlgpsinfo.execute(true);
}
void UAV_Database::InsertHEADING_LOCAL(double yaw, double pitch, double roll)
{
	sql << "CREATE TABLE IF NOT EXISTS heading_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, yaw double NOT NULL, pitch double NOT NULL, roll double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
	statement sqlheadinginfo = (sql.prepare << "INSERT INTO heading_local (yaw,pitch,roll,pulled) VALUES (:f,:f,:f,:b)", use(yaw), use(pitch), use(roll),use(0));
	sqlheadinginfo.execute(true);
}
void UAV_Database::InsertHEADING_ALT(double yaw, double pitch, double roll)
{
	sql << "CREATE TABLE IF NOT EXISTS heading_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, yaw double NOT NULL, pitch double NOT NULL, roll double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
	statement sqlheadinginfo = (sql.prepare << "INSERT INTO heading_alt (yaw,pitch,roll,pulled) VALUES (:f,:f,:f,:b)", use(yaw), use(pitch), use(roll),use(0));
	sqlheadinginfo.execute(true);
}
void UAV_Database::InsertWAYPOINT_LOCAL(double altitude, double latitude, double longitude)
{
	sql << "CREATE TABLE IF NOT EXISTS waypoint_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
	statement sqlwaypointinfo = (sql.prepare << "INSERT INTO waypoint_local (altitude,latitude,longitude,pulled) VALUES (:f,:f,:f,:b)", use(altitude), use(latitude), use(longitude),use(0));
	sqlwaypointinfo.execute(true);
}
void UAV_Database::InsertWAYPOINT_ALT(double altitude, double latitude, double longitude)
{
	sql << "CREATE TABLE IF NOT EXISTS waypoint_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
	statement sqlwaypointinfo = (sql.prepare << "INSERT INTO waypoint_alt (altitude,latitude,longitude,pulled) VALUES (:f,:f,:f,:b)", use(altitude), use(latitude), use(longitude),use(0));
	sqlwaypointinfo.execute(true);
}
void UAV_Database::InsertTARGET_LOCAL(double altitude, double latitude, double longitude)
{
	sql << "CREATE TABLE IF NOT EXISTS target_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP)";
	statement sqltargetinfo = (sql.prepare << "INSERT INTO target_local (altitude,latitude,longitude) VALUES (:f,:f,:f)", use(altitude), use(latitude), use(longitude));
	sqltargetinfo.execute(true);
}
void UAV_Database::PullGPS_LOCAL(double a[3],int pulled)
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	int tempid = 0;
	sql << "SELECT id,altitude,latitude,longitude FROM gps_local WHERE pulled = 0 LIMIT 1;", into(tempid), into(tempalt),into(templat),into(templon);
	if(pulled==0)
		sql << "UPDATE gps_local SET pulled = 1 WHERE id = :row",use(tempid);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_Database::PullGPS_ALT(double a[3],int pulled)
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	int tempid = 0;
	sql << "SELECT id,altitude,latitude,longitude FROM gps_alt WHERE pulled = 0 LIMIT 1;", into(tempid), into(tempalt),into(templat),into(templon);
	if(pulled==0)
		sql << "UPDATE gps_alt SET pulled = 1 WHERE id = :row",use(tempid);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_Database::PullHEADING_LOCAL(double a[3], int pulled)
{
	double tempyaw = 0;
	double temppit = 0;
	double temprol = 0;
	int tempid = 0;
	sql << "SELECT id,yaw,pitch,roll FROM heading_local WHERE pulled = 0 LIMIT 1;", into(tempid), into(tempyaw),into(temppit),into(temprol);
	if(pulled==0)
		sql << "UPDATE heading_local SET pulled = 1 WHERE id = :row",use(tempid);
	a[0] = tempyaw;
	a[1] = temppit;
	a[2] = temprol;
}
void UAV_Database::PullHEADING_ALT(double a[3], int pulled)
{
	double tempyaw = 0;
	double temppit = 0;
	double temprol = 0;
	int tempid = 0;
	sql << "SELECT id,yaw,pitch,roll FROM heading_alt WHERE pulled = 0 LIMIT 1;", into(tempid), into(tempyaw),into(temppit),into(temprol);
	if(pulled==0)
		sql << "UPDATE heading_alt SET pulled = 1 WHERE id = :row",use(tempid);
	a[0] = tempyaw;
	a[1] = temppit;
	a[2] = temprol;
}
void UAV_Database::PullWAYPOINT_LOCAL(double a[3], int pulled)
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	int tempid = 0;
	sql << "SELECT id,altitude,latitude,longitude FROM waypoint_local WHERE pulled = 0 LIMIT 1", into(tempid), into(tempalt),into(templat),into(templon);
	if(pulled==0)
		sql << "UPDATE waypoint_local SET pulled = 1 WHERE id = :row",use(tempid);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_Database::PullWAYPOINT_ALT(double a[3], int pulled)
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	int tempid = 0;
	sql << "SELECT id,altitude,latitude,longitude FROM waypoint_alt WHERE pulled = 0 LIMIT 1", into(tempid), into(tempalt),into(templat),into(templon);
	if(pulled==0)
		sql << "UPDATE waypoint_alt SET pulled = 1 WHERE id = :row",use(tempid);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_Database::PullTARGET_LOCAL(double a[3], int *t_id)
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	sql << "SELECT id,altitude,latitude,longitude FROM target_local WHERE id = (select max(id) from target_local);", into(*t_id), into(tempalt),into(templat),into(templon);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_Database::PullLatestGPS_LOCAL(double a[3])
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	sql << "SELECT altitude,latitude,longitude FROM gps_local WHERE id = (select max(id) from gps_local);", into(tempalt),into(templat),into(templon);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_Database::PullLatestGPS_ALT(double a[3])
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	sql << "SELECT altitude,latitude,longitude FROM gps_alt WHERE id = (select max(id) from gps_alt);", into(tempalt),into(templat),into(templon);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_Database::PullLatestHEADING_LOCAL(double a[3])
{
	double tempyaw = 0;
	double temppit = 0;
	double temprol = 0;
	sql << "SELECT yaw,pitch,roll FROM heading_local WHERE id = (select max(id) from heading_local);", into(tempyaw),into(temppit),into(temprol);
	a[1] = temppit;
	a[2] = temprol;
}
void UAV_Database::PullLatestHEADING_ALT(double a[3])
{
	double tempyaw = 0;
	double temppit = 0;
	double temprol = 0;
	sql << "SELECT yaw,pitch,roll FROM heading_alt WHERE id = (select max(id) from heading_alt);", into(tempyaw),into(temppit),into(temprol);
	a[0] = tempyaw;
	a[1] = temppit;
	a[2] = temprol;
}
void UAV_Database::PullLatestWAYPOINT_LOCAL(double a[3])
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	sql << "SELECT altitude,latitude,longitude FROM waypoint_local WHERE id = (select max(id) from waypoint_local);", into(tempalt),into(templat),into(templon);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_Database::PullLatestWAYPOINT_ALT(double a[3])
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	sql << "SELECT altitude,latitude,longitude FROM waypoint_alt WHERE id = (select max(id) from waypoint_alt);", into(tempalt),into(templat),into(templon);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_Database::DropGPS_LOCAL()
{
	sql << "DROP TABLE IF EXISTS gps_local";
}
void UAV_Database::DropGPS_ALT()
{
	sql << "DROP TABLE IF EXISTS gps_alt";
}
void UAV_Database::DropHEADING_LOCAL()
{
	sql << "DROP TABLE IF EXISTS heading_local";
}
void UAV_Database::DropHEADING_ALT()
{
	sql << "DROP TABLE IF EXISTS heading_alt";
}
void UAV_Database::DropWAYPOINT_LOCAL()
{
	sql << "DROP TABLE IF EXISTS waypoint_local";
}
void UAV_Database::DropWAYPOINT_ALT()
{
	sql << "DROP TABLE IF EXISTS waypoint_alt";
}
void UAV_Database::DropTARGET_LOCAL()
{
	sql << "DROP TABLE IF EXISTS target_local";
}
void UAV_Database::Query(string query)
{
	sql << query;
}
void UAV_Database::DatabaseDisconnect()
{
	sql.close();
}
void UAV_Database::InsertPixelTARGET(int x, int y)
{
	sql << "CREATE TABLE IF NOT EXISTS pixeltarget (id int NOT NULL PRIMARY KEY, x INT NOT NULL, y INT NOT NULL)";
	statement sqlpixeltargetinfo = (sql.prepare << "REPLACE INTO pixeltarget (id,x,y) VALUES (1,:i,:i)", use(x), use(y));
	sqlpixeltargetinfo.execute(true);
}
void UAV_Database::PullPixelTARGET(int xy[2])
{
	sql << "SELECT x,y FROM pixeltarget where id = 1;", into(xy[0]),into(xy[1]);
}
void UAV_Database::SetMissionStatus(int status)
{
	sql << "CREATE TABLE IF NOT EXISTS missionstatus (id int NOT NULL PRIMARY KEY, status INT NOT NULL)";
	statement sqlstatusinfo = (sql.prepare << "REPLACE INTO missionstatus (id,status) VALUES (1,:i)", use(status));
	sqlstatusinfo.execute(true);
}

int UAV_Database::CheckMissionStatus()
{
	int status = 0;
	sql << "SELECT status FROM missionstatus WHERE id = 1", into(status);
	return status;
}

void UAV_Database::SetTargetStatus(int status)
{
    sql << "CREATE TABLE IF NOT EXISTS targetstatus (id int NOT NULL PRIMARY KEY, status INT NOT NULL)";
    statement sqlstatusinfo = (sql.prepare << "REPLACE INTO targetstatus (id,status) VALUES (1,:i)", use(status));
    sqlstatusinfo.execute(true);
}

int UAV_Database::CheckTargetStatus()
{
    int status = 0;
    sql << "SELECT status FROM targetstatus WHERE id = 1", into(status);
    return status;
}



int UAV_Database::flightdatamigration()
{
    int past_flight_id;
    int current_flight_id;
    sql << "SELECT flight_id FROM flight_id WHERE id = 1", into(past_flight_id);
    current_flight_id = past_flight_id + 1;
    statement sqlflight_id = (sql.prepare << "REPLACE INTO flight_id (id,flight_id) VALUES (1,:i)", use(current_flight_id));
    sqlflight_id.execute(true);
    sql << "INSERT INTO ach_gps_local (flight_id,altitude,latitude,longitude,time) SELECT :i,altitude,latitude,longitude,time FROM gps_local;", use(past_flight_id);
    sql << "INSERT INTO ach_gps_alt (flight_id,altitude,latitude,longitude,time) SELECT :i,altitude,latitude,longitude,time FROM gps_alt;", use(past_flight_id);
    sql << "INSERT INTO ach_heading_local (flight_id,yaw,pitch,roll,time) SELECT :i,yaw,pitch,roll,time FROM heading_local;", use(past_flight_id);
    sql << "INSERT INTO ach_heading_alt (flight_id,yaw,pitch,roll,time) SELECT :i,yaw,pitch,roll,time FROM heading_alt;", use(past_flight_id);
    sql << "INSERT INTO ach_waypoint_local (flight_id,altitude,latitude,longitude,time) SELECT :i,altitude,latitude,longitude,time FROM waypoint_local;", use(past_flight_id);
    sql << "INSERT INTO ach_waypoint_alt (flight_id,altitude,latitude,longitude,time) SELECT :i,altitude,latitude,longitude,time FROM waypoint_alt;", use(past_flight_id);
    sql << "INSERT INTO ach_target_local (flight_id,altitude,latitude,longitude,time) SELECT :i,altitude,latitude,longitude,time FROM target_local;", use(past_flight_id);
    sql << "INSERT INTO ach_target_alt (flight_id,altitude,latitude,longitude,time) SELECT :i,altitude,latitude,longitude,time FROM target_alt;", use(past_flight_id);
    sql << "TRUNCATE gps_local;";
    sql << "TRUNCATE gps_alt;";
    sql << "TRUNCATE heading_local;";
    sql << "TRUNCATE heading_alt;";
    sql << "TRUNCATE waypoint_local;";
    sql << "TRUNCATE waypoint_alt;";
    sql << "TRUNCATE target_local;";
    sql << "TRUNCATE target_alt;";
    return current_flight_id;
}

void UAV_Database::InsertImageString_S(string image)
{
	sql << "CREATE TABLE IF NOT EXISTS image_strings_s (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, token text NOT NULL)";
	statement sqlimageinfo = (sql.prepare << "INSERT INTO image_strings_s (token) VALUES (:t)", use(image));
	sqlimageinfo.execute(true);
}

string UAV_Database::PullImageString_S(int id)
{
	string buffer;
	sql << "SELECT token FROM image_strings_s WHERE id = :i", use(id),into(buffer);
	return buffer;
}

int UAV_Database::ImageStringNum_S()
{
	int buffer;
	sql << "SELECT id FROM image_strings_s ORDER BY id DESC LIMIT 0,1", into(buffer);
	return buffer;
}

void UAV_Database::InsertImageString_R(string image, int target_id)
{
	sql << "CREATE TABLE IF NOT EXISTS image_strings_r (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, token text NOT NULL, target_id int NOT NULL)";
	statement sqlimageinfo = (sql.prepare << "INSERT INTO image_strings_r (token) VALUES (:t)", use(image));
	sqlimageinfo.execute(true);
}

string UAV_Database::PullImageString_R(int id, int target_id)
{
	string buffer;
	sql << "SELECT token FROM image_strings_r WHERE id = :i AND target_id = :i", use(id), use(target_id),into(buffer);
	return buffer;
}

int UAV_Database::ImageStringNum_R()
{
	int buffer;
	sql << "SELECT id FROM image_strings_r ORDER BY id DESC LIMIT 0,1", into(buffer);
	return buffer;
}

void UAV_Database::ClearImageString_S()
{
    sql << "TRUNCATE image_strings_s;";
}

void UAV_Database::ClearImageString_R()
{
	sql << "TRUNCATE image_strings_r;";
}

void UAV_Database::InsertMission(std::string msg)
{
    sql << "REPLACE INTO missionjson (id,msg) VALUES (1,'" << msg << "');";
}

std::string UAV_Database::PullMission()
{
    std::string buffer;
    sql << "SELECT msg FROM missionjson WHERE id=1", into(buffer);
    return buffer;
}
