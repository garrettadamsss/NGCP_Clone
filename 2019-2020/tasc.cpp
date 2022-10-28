/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tasc.cpp (Tristan add Sherman Cady)
 *
 * @brief Task machine (manager) for the flight computer . 
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Tristan Sherman
 * @author Tristan Cady
 *
 *
 * Further edits:
 * 2020
 * Dakota Eckheart, NGCP Cal Poly Pomona <kodyeckheart@gmail.com>
 *
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "tasc.h"
#include <fstream>

using json = nlohmann::json;

int WP_RADIUS = 20;
float first_distance = 30;
float second_distance = 100;
float angle = 20; //Must be moved to config file later
int database = 1;
float target_radius = 5; //meters

//1: (close door/drop pin, open doors, open pin)
//2: 
int mission = 1;


int door1 = 9;
int door2 = 10;
int pin = 8; //Servo used for payload drop servo
int drop_open = 1300; //PWM used to activate servo for payload drop
int drop_close = 1000;
int door1_close = 810;
int door1_open = 1570;
int door2_close = 830;
int door2_open = 1650;
uint16_t message;
mutex isr_mutex;

//Initiallize database
UAV_Database uav_db;
jsontomission jmission;



int mission_status;
bool isr_visible = true;
double gps_info[3] = {0.0, 0.0, 0.0};
double yaw = 0;

// ------------------------------------------------------------------------------
//   INITIALIZATION
// ------------------------------------------------------------------------------
int
top(int argc, char **argv) {

    // --------------------------------------------------------------------------
    //   PARSE THE COMMANDS
    // --------------------------------------------------------------------------

    // Default input arguments
#ifdef __APPLE__
    char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
    char *uart_name = (char *) "/dev/ttyACM0";
#endif
    int baudrate = 115200;

    // do the parse, will throw an int if it fails
    parse_commandline(argc, argv, uart_name, baudrate);




    // --------------------------------------------------------------------------
    //   PORT and THREAD STARTUP
    // --------------------------------------------------------------------------

    /*
     * Instantiate a serial port object
     *
     * This object handles the opening and closing of the offboard computer's
     * serial port over which it will communicate to an autopilot.  It has
     * methods to read and write a mavlink_message_t object.  To help with read
     * and write in the context of pthreading, it gaurds port operations with a
     * pthread mutex lock.
     *
     */
    Serial_Port serial_port(uart_name, baudrate);


    /*
     * Instantiate an autopilot interface object
     *
     * This starts two threads for read and write over MAVlink. The read thread
     * listens for any MAVlink message and pushes it to the current_messages
     * attribute.  The write thread at the moment only streams a position target
     * in the local NED frame (mavlink_set_position_target_local_ned_t), which
     * is changed by using the method update_setpoint().  Sending these messages
     * are only half the requirement to get response from the autopilot, a signal
     * to enter "offboard_control" mode is sent by using the enable_offboard_control()
     * method.  Signal the exit of this mode with disable_offboard_control().  It's
     * important that one way or another this program signals offboard mode exit,
     * otherwise the vehicle will go into failsafe.
     *
     */
    Autopilot_Interface autopilot_interface(&serial_port);

    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    serial_port_quit = &serial_port;
    autopilot_interface_quit = &autopilot_interface;
    signal(SIGINT, quit_handler);

    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port.start();

    autopilot_interface.messages_to_read.read_global_position_int = true;
    autopilot_interface.messages_to_read.read_heartbeat = true;
    autopilot_interface.messages_to_read.read_attitude = true;

    autopilot_interface.start();

    //---------------------------------------------------------------------------
    //START COLLISION AVOIDANCE THREAD
    //---------------------------------------------------------------------------
    printf("START COLLISION AVOIDANCE\n");
    autopilot_interface.start_collision_avoidance();

    // --------------------------------------------------------------------------
    //  BEGIN FLIGHT COMPUTER CONTROL
    // --------------------------------------------------------------------------

    /*
     * Now we can implement the algorithm we want on top of the autopilot interface
     */
    TASC(autopilot_interface);


    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------

    /*
     * Now that we are done we can stop the threads and close the port
     */
    autopilot_interface.stop();
    serial_port.stop();


    // --------------------------------------------------------------------------
    //   DONE
    // --------------------------------------------------------------------------

    // woot!
    return 0;

}


void run_isr_thread(){
    ISR isr;
    isr.start(&isr_visible, &isr_mutex, gps_info, &yaw, &mission_status);
}



// ------------------------------------------------------------------------------
//   TASC FLIGHT MODE MANAGER
// ------------------------------------------------------------------------------
/* This function controls which flight task is run based off of what the database
   is reading from the communication network
*/



void
TASC(Autopilot_Interface &api) {

    //START DATABASE
    uav_db.Connect("plane1", "root", "ngcp");



    //waypointGenerator(api, wp1, wp3, 3, wp2, 75, 15);
    //printf("Done writing waypoints\n");
    //sleep(100);



    uint16_t task = api.current_messages.rc_channels_t.chan6_raw;

    double lat_takeoff = 33.9322558;
    double lon_takeoff = -117.6334101;

    double lat_loiter = 33.9331104;
    double lon_loiter = -117.6325089;

    double lat_landpoint = 33.9322847;
    double lon_landpoint = -117.6317337;

    double lat_landset = 33.9332038;
    double lon_landset = -117.6288879;

    thread isr_thread(run_isr_thread);
    int new_mission_status = -1;

    while (true) {

        /** @TODO
         *  
         * Determine how to eliminate database as the middleman between this code
         * and the Xbee packets that GCS sends.
         * 
         */
	    new_mission_status = uav_db.CheckMissionStatus();
        json missioninfo = json::parse(uav_db.PullMission());
        jmission.fillMission(missioninfo);  // mission object declared globally so all functions can access
        int perform_stage = missioninfo["perform_stage"];

        /*-----------------------------------------------------------------------------------
            LOITER
        -----------------------------------------------------------------------------------*/
        if (missioninfo["stop"] == true) {
            loiter(api);
            printf("Looks like you want to loiter! Nah dude, not this year\n");
        }

        //If message has changed then we will change the state
        else if (mission_status != new_mission_status) {
            printf("Mission Change\n");
            mission_status = uav_db.CheckMissionStatus();
            
            isr_mutex.lock();
            if(mission_status == 99){   /* TODO: Needs valid value */
                uav_db.PullLatestGPS_LOCAL(gps_info);
            }
            isr_mutex.unlock();

            

            /*-----------------------------------------------------------------------------------
                WAIT
            -----------------------------------------------------------------------------------*/
            else if (perform_stage == 0) {
                printf("Waiting and Ready");

            }

            /*-----------------------------------------------------------------------------------
                TAKEOFF
            -----------------------------------------------------------------------------------*/
            if (perform_stage == 1) {
                takeoff(api);  		
                printf("Takeoff\n");
            }

            //TODO verify if this ISR logic works for radio location, not just image recognition
            else if (perform_stage == 2) {
                double wp1[2] = {jmission.isr.topleft_lat, jmission.isr.topleft_lon}; 
                double wp2[2] = {jmission.isr.botleft_lat, jmission.isr.botleft_lon};
                double wp3[2] = {jmission.isr.botright_lat, jmission.isr.botright_lon};

                //Generate search points
                waypointGenerator(api, wp1, wp3, 4, wp2, jmission.isr.search_alt, 20);
                
                //Send command to start mission
                isr_mutex.lock();
                isr_visible = true;
                isr_mutex.unlock();
            }

            /*-----------------------------------------------------------------------------------
                PAYLOAD DROP
            -----------------------------------------------------------------------------------*/
            else if (perform_stage == 3) {
                    printf("Payload Drop\n");
                    payload_drop(api);
            }

            /*-----------------------------------------------------------------------------------
                LAND
            -----------------------------------------------------------------------------------*/
            else if (perform_stage == 9) {      
                land(api);
                printf("Land yourself ya dingo\n");
            }
             
            else {
                printf("Mission variable not defined\n");
            }

            mission_status = new_mission_status;

        }//End if statement
        sleep(1);
    }// End while loop
    return;

}

/***
TODO: Implement nofly_zone function to avoid exiting the geofence

  Needed:
  	Geofence coordinates
	
----------------------------------------------------------------------------------
CREATE STARTING AND LOITER WAYPOINTS TO AVOID NO-FLY ZONE
-----------------------------------------------------------------------------------
void
no_fly_zone(AutoPilot_Interface &api) {
    std::vector<mavlink_mission_item_t> waypoints;
    
    waypoints.push_back(api.create_waypoint(0,0,0,0,0);
    waypoints.push_back(api.create_waypoint(_,_,_,2,WP_RADIUS));	//TODO: create valid waypoint
    waypoints.push_back(api.create_loiter_point(_,_,_,3,WP_RADIUS));	//TODO: "	"	"
    
    api.write_waypoints(waypoints, 1);
    
    return;
}
***/

/*----------------------------------------------------------------------------------
CREATE TAKEOFF AND LOITER WAYPOINTS
-----------------------------------------------------------------------------------*/
void
takeoff(Autopilot_Interface &api) {

    std::vector<mavlink_mission_item_t> waypoints;

    waypoints.push_back(api.create_waypoint(0,0,0,0,0);
    waypoints.push_back(api.create_takeoff_point(0,0,40,1));
    waypoints.push_back(api.create_waypoint(jmission.tkf.takeoff_lat,
                                            jmission.tkf.takeoff_lon,
                                            jmission.tkf.takeoff_alt,
                                            2,
                                            jmission.tkf.loiter_radius));
    waypoints.push_back(api.create_loiter_point(jmission.tkf.loiter_lat,
                                                jmission.tkf.loiter_lon,
                                                jmission.tkf.loiter_alt,
                                                3,
                                                jmission.tkf.loiter_radius));
		    
    api.write_waypoints(waypoints, 1);

    sleep(1);

    api.arm_disarm(1);


    sleep(2);

    api.setMode("AUTO");
    return;
}



/*----------------------------------------------------------------------------------
CREATE A LOITER POINT
-----------------------------------------------------------------------------------*/
void
loiter(Autopilot_Interface &api) {

    std::vector<mavlink_mission_item_t> waypoints;

    waypoints.push_back(api.create_waypoint(0, 0, 0, 0, 0));
    waypoints.push_back(api.create_loiter_point(jmission.ltr.loiter_lat, 
                                                jmission.ltr.loiter_lon,
                                                jmission.ltr.loiter_alt, 
                                                1, 
                                                jmission.ltr.loiter_radius));
    api.write_waypoints(waypoints, 1);

    return;
}



/*----------------------------------------------------------------------------------
LAND AT SPECIFIC LOCATION
-----------------------------------------------------------------------------------*/
void
land(Autopilot_Interface &api) {
//First coordinates specify the final waypoint before landing is initialized
// This coordinate should be lined up with the runway
// Second coordinates specify where the aircraft will touch down.

    std::vector<mavlink_mission_item_t> waypoints;

    // Setup WP --> Land point
    waypoints.push_back(api.create_waypoint(0, 0, 0, 0, 0));
    waypoints.push_back(api.create_waypoint(jmission.lnd.first_lat,
                                            jmission.lnd.first_lon, 
                                            jmission.lnd.first_alt, 
                                            1, 
                                            WP_RADIUS));
    waypoints.push_back(api.create_land_point(jmission.lnd.second_lat,
    					                        jmission.lnd.second_lon, 
					                            2));
    api.write_waypoints(waypoints, 1);

    return;
}



/*------------------------------------------------------------------------------------
CALCULATE DROP DISTANCE AND DROP PAYLOAD
------------------------------------------------------------------------------------*/
void
payload_drop(Autopilot_Interface &api) {

    /*
      * Polling target location --
      */    
    double target[3];
    target[0] = (double)jmission.pid.first_alt;
    target[1] = (double)jmission.pid.first_lat;
    target[2] = (double)jmission.pid.first_lon;
    double plane_lat = api.current_messages.global_position_int.lat / 1E7,
            plane_long = api.current_messages.global_position_int.lon / 1E7,
            plane_alt = api.current_messages.global_position_int.relative_alt / 1E3;

    /*
     * Payload_Drop object from NGCPPayloadDrop
     
	Changed variable name from "payload_drop" to "pay_drop" due to function name
     */

    uint16_t one = 1;
    Payload_Drop pay_drop(jmission.pid.first_lat, 
                            jmission.pid.first_lon, 
                            plane_lat, 
                            plane_long, 
                            plane_alt);
    /*
     * Create 3 waypoints at an angle
     *
     *    (First_wp)--------100m--------(target_wp)---30m---(Third_wp)
     *
     */

    mavlink_mission_item_t angleDefine = api.distanceVectors(jmission.pid.second_lat,
    							     jmission.pid.second_lon, 
							     jmission.pid.first_lat, 
							     jmission.pid.first_lon);

    //Convert from radians to degrees
    angle = atan2(angleDefine.x, angleDefine.y) * 180/3.1415;

    vector<Waypoint> pay_waypoints = pay_drop.payload_waypoints(100, 30, angle);


    vector<mavlink_mission_item_t> waypoints;
    // For some reason WP 0(HOME) needs to be sent but need different command to actually change HOME ???
    waypoints.push_back(api.create_waypoint(0, 0, 0, 0, 0));//Dummy data that is required* but not used
    waypoints.push_back(api.create_waypoint(pay_waypoints[0].lat, pay_waypoints[0].lon, 35, 1, WP_RADIUS));
    waypoints.push_back(api.create_waypoint(pay_waypoints[1].lat, pay_waypoints[1].lon, 35, 2, WP_RADIUS));
    waypoints.push_back(api.create_waypoint(pay_waypoints[2].lat, pay_waypoints[2].lon, 35, 3, WP_RADIUS));


    /*
     * Send waypoints to pixhawk
     */
    api.write_waypoints(waypoints, 1);
    api.setCurrentWaypoint(one);

    /*
     * Staring Payload mission
     */

    //We must first get to first_wp
    bool at_first_wp = false;
    
    while (!at_first_wp) {

        usleep(200000); //check and update gps 5 times a second
        //Updating our GPS location
        double distance = pay_drop.gpsDistance(pay_waypoints[0].lat,
                                                   pay_waypoints[0].lon, //While we are not at first_wp
                                                   api.current_messages.global_position_int.lat / 1E7,
                                                   api.current_messages.global_position_int.lon / 1E7);
        printf("Distance to first wp: %f\n", distance);

	//Break out of pd	
	    if (uav_db.CheckMissionStatus() != 4) {return;}

            if (distance < WP_RADIUS) {
            at_first_wp = true;

	

            }
    }


    printf("Reached first waypoint!!!\n");
    api.write_set_servo(door1, door1_open);
    api.write_set_servo(door2, door2_open);

    /*
     * Calculating time to drop
     *
     */
    double vx = (double) api.current_messages.global_position_int.vx / 100;
    double vy = (double) api.current_messages.global_position_int.vy / 100;
    double speed = sqrt(vx * vx + vy * vy);
    double alt = (float) api.current_messages.global_position_int.relative_alt / 1000;
    double curLoc[3] = {0,
                        api.current_messages.global_position_int.lat / 1E7,
                        api.current_messages.global_position_int.lon / 1E7};
    double heading = api.current_messages.global_position_int.hdg / 100;
    NGCPPayloadDrop drop(alt, speed, 0.0);

    double timeToDrop = drop.timeToDrop(target, curLoc, heading, speed) - 0.5;

    cout << "Time to drop payload: " << timeToDrop << endl;
    cout << "Expecting to drop at:" << alt << " meters" << endl;
    chrono::steady_clock::time_point start = chrono::steady_clock::now();





    /*
     * check to see if we are near target then drop payload
     */
    bool nearTarget = false;
    while (!nearTarget) { //while we are not near the target

        usleep(25000); //check and update gps 40 times a second

        //Updating our GPS location and check for collisions

        double distance = pay_drop.gpsDistance(pay_drop.TLat, pay_drop.TLong, //While we are not at first_wp
                                                api.current_messages.global_position_int.lat / 1E7,
                                                api.current_messages.global_position_int.lon / 1E7);
        printf("Distance to first target: %f\n", distance);

        chrono::steady_clock::time_point end = chrono::steady_clock::now();
        chrono::steady_clock::duration time = end - start;
        if (time > chrono::milliseconds((int) (timeToDrop * 1000))) {
            typedef chrono::duration<double> double_seconds;
            auto time_f = chrono::duration_cast<double_seconds>(time);
            cout << "Time drop was executed: " << time_f.count() << endl;
            if (distance < target_radius) {
                nearTarget = true;
                api.write_set_servo(pin, drop_open);
            }
        }
    }

    printf("Dropped Payload!!!!!\n");
    sleep(1);

    api.write_set_servo(pin, drop_close);
    api.write_set_servo(door1, door1_close);
    api.write_set_servo(door2, door2_close);

    return;
}



// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate) {

    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n", commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                baudrate = atoi(argv[i + 1]);

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

    }
    // end: for each input argument

    // Done!
    return;
}



// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler(int sig) {
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    // autopilot interface
    try {
        autopilot_interface_quit->handle_quit(sig);
    }
    catch (int error) {}

    // serial port
    try {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error) {}

    // end program here
    exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv) {
    // This program uses throw, wrap one big try/catch here
	printf("hello");
    try {
        int result = top(argc, argv);
        return result;
    }

    catch (int error) {
        fprintf(stderr, "mavlink_control threw exception %i \n", error);
        return error;
    }

}

