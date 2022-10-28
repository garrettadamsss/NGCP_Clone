//David Douglas
//The following program is converted from matlab to c++

#include "NGCPPayloadDrop.h"

using namespace std;
using namespace boost::numeric::odeint;

int i=0;
int wind;
double *x = new double[50];
double *y = new double[50];

typedef std::array<double,4> state_type;



NGCPPayloadDrop::NGCPPayloadDrop(double a, double v, double w):vel(v),alt(a),w1(w){}

double NGCPPayloadDrop::toRadians(float deg)
{
    return 3.1415926535897 * deg/180.0;
}

double NGCPPayloadDrop::distanceFromBall(float x1, float y1, float curLat, float curLong)
{
        float deltaX = curLat - x1;
        float deltaY = curLong - y1;
        float oneLat = 111132.954 - 559.822*cos(2*toRadians(curLat))+1.175*cos(4*toRadians(curLat));
        float oneLong = toRadians(6378137.0)*cos(toRadians(curLat));
        float xMeters = deltaX*oneLat;
        float yMeters = deltaY*oneLong;
        return (double)sqrt(xMeters*xMeters + yMeters*yMeters);
}

double NGCPPayloadDrop::timeToDrop(double *target, double *curLocation, double heading, double speed)
{
        //assuming the plane is going straight in the direction of the ball
        double distAway = xvalue();
        double directOfDropspot = 180 + heading;
        while(directOfDropspot > 360) directOfDropspot -= 360;
        double radians;
        if(heading < 90)radians = 90.0-heading;
        else if(heading < 180) radians = 360.0 - (heading - 90);
        else if(heading < 270) radians = 270.0 - (heading - 180);
        else radians = 180.0 - (heading - 270);
        double xDist = sin(radians/180 * 3.1415926535897)*distAway;
        double yDist = cos(radians/180 * 3.1415926535897)*distAway;
        double dropRadius = sqrt(xDist*xDist + yDist*yDist);
        double distToDrop = distanceFromBall(target[1],target[2],curLocation[1],curLocation[2]) - dropRadius;
        return distToDrop/speed;//assuming speed is in meters per second
}


void eof( const state_type &z , state_type &zdot ,  double t ){
        const double g = 9.81;               // acceleration due to gravity constant  (m/s^2)
    const double m = .56;               //mass of cannonball (kg)
    const double c = 0.7;               // drag coefficient
    const double rho = 1.29;             //air density (kg/m^3)
    const double s= .015677388;         // projectile cross-sectional area (m^2)
    double w = wind;
    //all of these are in polar coordinates
    zdot[0] = z[3]*cos(z[2]);//x velocity wrt time
    zdot[1] = z[3]*sin(z[2]);//y velocity wrt time
    double D = c*rho*(s/2)*(((zdot[0])-w)*((zdot[0])-w)+zdot[1]*zdot[1]);//drag equation
    zdot[2] = -g/z[3]*cos(z[2]);//change in theta wrt time
    zdot[3] = -D/m-g*sin(z[2]);//change in velocity wrt time
}
void printnstore(const state_type &z , const double t){
    x[i] = z[0];
    y[i] = z[1];
    i++;
}
double NGCPPayloadDrop::xvalue(){
    wind = w1;
    typedef runge_kutta_dopri5<state_type> step;//stepper, basically the ODE fuction needs it
    int trials = 50;// this value wont need to be over 50 unless the plane is 120 meters in the air
    double theta = 0;//angle in radians
    state_type z = {0,alt,theta,vel};//essentially a double array that the integrator can handle
    integrate_const(make_dense_output<step>( 1E-6, 1E-3 ),eof,z,0.0,(double)(trials/10),0.1,printnstore);//OED45 function in c++

    int l =0;
    while(l<trials){
    if(*(y+l) < 0)
        break;
    else
        l++;
}
    double a1 = x[l-1];
    double a2 = x[l];
    double b1 = y[l-1];
    double b2 = y[l];
    double slope = (b2-b1)/(a2-a1);
    double c = b1-slope*a1;//b = y-mx
    double X = -c/slope;//mx+b=0 b/m
    return X;
}

/*
 * Constructor
 */
Payload_Drop::
Payload_Drop(double tlat, double tlong, double plat, double plong, double a) {
    this->TLat = tlat;
    this->TLong = tlong;
    PLat = plat;
    PLong = plong;
    altitude = a;
}
/*
 * Change plane GPS( lat, lon, alt)
 */
void
Payload_Drop::
setGPSinfo(double plat, double plong, double a)
{
    PLat = plat;
    PLong = plong;
    altitude = a;
}

/*
 * Calculates time it will take to drop target -- TODO input code but first fix Douglas' NGCPPayloadDrop.cpp
 */
double
Payload_Drop::
timeToDrop()
{
    return 0;
}

/*
 * Set Velocity given vector components
 */
void
Payload_Drop::
setVelocity(int vx, int vy, int vz)
{
    velocity = sqrt(vx*vx) + sqrt(vy*vy) + sqrt(vz*vz);
}


/*
 * Rudimentary code to drop the payload when we are near the target. Last resort if timeToDrop is not working
 * @param radius: in meters how close do we get to target before dropping payload
 */
bool
Payload_Drop::
near_target(Autopilot_Interface &api, const int &radius) {
    bool nearTarget = false;
    printf("Radius: %d\n", radius);
    int i = 0;
    while(!nearTarget){
        //Update GPS
        setGPSinfo(api.current_messages.global_position_int.lat / 1E7,
                   api.current_messages.global_position_int.lon / 1E7,
                   api.current_messages.global_position_int.relative_alt / 1E3);
        double dist = gpsDistance(TLat, TLong, PLat, PLong);
        printf("Distance to target: %f\n",dist);
        if(radius >= dist) break;
        usleep(1000000);
    }
    return true;




}

// --------------------------------------------------------------------------------------
// Helper Functions for payload drop
// --------------------------------------------------------------------------------------

/*
	Calculates the distance between the target's latitude and longitude coordinates and the plane's latitude and longitude coordinates
	Calculates in meters
*/
double
Payload_Drop::
gpsDistance(const double &target_lat, const double &target_long, const double &current_lat, const double &current_long)
{
    double tLatRad = target_lat * TO_RADIANS;
    double pLatRad = current_lat * TO_RADIANS;
    double deltaLat = (current_lat - target_lat) * TO_RADIANS;
    double deltaLong = (current_long - target_long) * TO_RADIANS;

    double var = sin(deltaLat / 2) * sin(deltaLat / 2) + (cos(tLatRad) * cos(pLatRad)) * (sin(deltaLong / 2) * sin(deltaLong / 2));
    double c = 2 * atan2(sqrt(var), sqrt(1 - var));

    return RADIUS_EARTH * c;
}

/*
 * Return the three waypoints needed to properly drop payload
 */
vector<Waypoint>
Payload_Drop::
payload_waypoints(const int &first_distance, const int &second_distance, const double &angle)
{
    //Initialize data strucutres
    vector<Waypoint> waypoints;
    Waypoint target(TLat,TLong,altitude);
    double angle_rad = angle * TO_RADIANS;

    //Find displacement of first waypoint. Think of it like a right triangle and we're finding the 'a' and 'b' sides
    double first_x_displacement = first_distance * cos(angle_rad);
    double first_y_displacement = first_distance * sin(angle_rad);

    //Find the waypoint given the x and y displacement of target
    Waypoint first_waypoint = meterDisplacement(first_x_displacement,first_y_displacement, target);
    waypoints.push_back(first_waypoint);

    //Our target is the middle waypoint
    waypoints.push_back(target);

    //Find displacement of first waypoint. Think of it like a right triangle and we're finding the 'a' and 'b' sides
    double second_x_displacement = -second_distance * cos(angle_rad);
    double second_y_displacement = -second_distance * sin(angle_rad);
    //Find the waypoint given the x and y displacement of target
    Waypoint second_waypoint = meterDisplacement(second_x_displacement,second_y_displacement, target);
    waypoints.push_back(second_waypoint);

    return waypoints;

}

/*Return a new position given the x and y displacement in meters of a waypoint*/
Waypoint
Payload_Drop::
meterDisplacement(const double & deltaX, const double & deltaY, const Waypoint & b){
    //coordinate offset in Radians
    float deltaLat = (deltaY / RADIUS_EARTH);
    float deltaLong = deltaX / (RADIUS_EARTH * cos(b.lat * PI / 180));

    Waypoint newPosition;
    newPosition.lat = b.lat + (deltaLat * (180 / PI));
    newPosition.lon = b.lon + deltaLong * ((180 / PI));
    return newPosition;
}
