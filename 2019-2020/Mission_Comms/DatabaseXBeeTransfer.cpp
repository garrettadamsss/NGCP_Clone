#include <iostream>
#include <istream>
#include <ostream>
#include <fstream>
#include <string>
#include <exception>
#include "UAV_Database.h"
#include <ctime>
#include <stdlib.h>
#include <time.h>
#include "include/commproto.h"
#include <thread>
#include <chrono>
#include <queue>
#include "callback_struct.h"
#include "include/VehicleGlobalPosition.hpp"
#include "include/VehicleWaypointCommand.hpp"
#include "include/VehicleInertialState.hpp"
#include "include/TargetDesignationCommand.hpp"
#include "unistd.h"
#include "limits.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

#define uav_val 6
#define uav_sig 2
#define gcs 1
#define comp 3
#define ugv 5

using namespace std;
using namespace comnet;

string db_name; // Local plane database connection info
string db_user; // Username of database login
string db_pass; // Password of database login
string usb_loc; // Location and name of usb
string dest_address; // Address of the XBee
string pollrate_seconds; // How fast to send new data in seconds
string pollrate_nanoseconds; // How fast to send new data in microseconds.
string target_pollrate; //How fast to send new target data;
string is_isrplane; // Whether or not this is the ISR Plane
string gcs_datastream; // Whether or not to connect to groundcontrol station
string gcs_dest_address; // Address ofhttps://www.google.com/search?client=ubuntu&channel=fs&q=jcpenny+returns&ie=utf-8&oe=utf-8 the ground control station XBee
string platform_id;
bool bad_config; // used to mark if config is not populated correctly

queue<callback_struct> callback_queue;

//Can be struct of class

struct db_mission_status : INHERITS_ABSPACKET {

    //Constructor
    db_mission_status(int status = 0)
        : CHAIN_ABSPACKET(db_mission_status) //passing class name
        , status(status)
        { }


    /**
       Pack data into the stream for sending out.
     */
    void Pack(ObjectStream &obj) override {
        obj << status;
        }

    /**
       Unpack data back into this packet when receiving data.
     */
    void Unpack(ObjectStream &obj) override {
        obj >> status;
        }

    /**
     Tells CommProtocol how to recreate the
     db_mission_status packet
     when receiving data.
     */
    AbstractPacket *Create() override {
    return new db_mission_status();
    }

    /**
       Data.
     */
    int status;
};//End struct


struct image_packet: INHERITS_ABSPACKET {

    //Constructor
    image_packet(string img_string = "")
            : CHAIN_ABSPACKET(image_packet) //passing class name
            , img_string(img_string)
    { }


    /**
       Pack data into the stream for sending out.
     */
    void Pack(ObjectStream &obj) override {
        obj << img_string;
    }

    /**
       Unpack data back into this packet when receiving data.
     */
    void Unpack(ObjectStream &obj) override {
        obj >> img_string;
    }

    /**
     Tells CommProtocol how to recreate the
     db_mission_status packet
     when receiving data.
     */
    AbstractPacket *Create() override {
        return new image_packet();
    }

    /**
       Data.
     */
    string img_string;
};//End struct



// Callback function that we will be using to link to our GPS datapacket.
// Essentially gets called when it recieves data from other xbee.

error_t db_mission_statusCallback(
const Header &header, db_mission_status &packet, Comms &node)
{
    cout << "=::RECEIVED MISSION STATUS PACKET::=		";
    cout << "Source node: " << (int32_t)header.source_id;
    cout << "   Message: ";
    cout << packet.status << endl;
    callback_struct status_struct = {header.source_id,packet.status,0,0,0,"",0};
    callback_queue.push(status_struct);
    return comnet::CALLBACK_SUCCESS | comnet::CALLBACK_DESTROY_PACKET;
}

error_t VehicleInertialStateCallback(const Header &header, ngcp::VehicleInertialState &packet, Comms &node)
{
    cout << "=::RECEIVED GLOBAL INERTIAL PACKET::=  ";
    cout << "Source node: " << (int32_t)header.source_id << endl;
    callback_struct inertial_struct = {packet.vehicle_id,packet.altitude,packet.latitude,packet.longitude,packet.heading,"",1};
    callback_queue.push(inertial_struct);
    return comnet::CALLBACK_SUCCESS | comnet::CALLBACK_DESTROY_PACKET;
}


error_t VehicleGlobalPositionCallback(
const Header &header, ngcp::VehicleGlobalPosition &packet, Comms &node)
{
    cout << "=::RECEIVED GLOBAL PACKET::=		";
    cout << "Source node: " << (int32_t)header.source_id;
    cout << "   Message: ";
    cout << packet.altitude << ", ";
    cout << packet.latitude << ", ";
    cout << packet.longitude;
    callback_struct gpos_struct = {packet.vehicle_id,packet.altitude ,packet.latitude,packet.longitude,0,"",4};
    callback_queue.push(gpos_struct);
    cout << "   !!!INSERTED!!!" << endl;
    return comnet::CALLBACK_SUCCESS | comnet::CALLBACK_DESTROY_PACKET;
}


error_t image_packetcallback(const Header &header, image_packet &packet, Comms &node)
{
    cout << "=::RECEIVED IMAGE PACKET PACKET::=		";
    cout << "Source node: " << (int32_t)header.source_id;
    cout << "   Message: ";
    cout << packet.img_string.size() << endl;
    callback_struct image_struct = {header.source_id,0,0,0,0,packet.img_string,3};
    callback_queue.push(image_struct);
    return comnet::CALLBACK_SUCCESS | comnet::CALLBACK_DESTROY_PACKET;
}

error_t TargetDesignationCommandCallback(const Header &header, ngcp::TargetDesignationCommand &packet, Comms &node)
{
    cout << "=::RECEIVED GLOBAL TARGET PACKET::=  ";
    cout << "Source node: " << (int32_t)header.source_id << endl;
    callback_struct target_struct = {packet.vehicle_id,packet.altitude,packet.latitude,packet.longitude,0,"",2};
    callback_queue.push(target_struct);
    return comnet::CALLBACK_SUCCESS | comnet::CALLBACK_DESTROY_PACKET;
}

int main()
{
    string line;

    // Pull in config data and store it in the designated strings for later use
    fstream cfgfile;
    cfgfile.open("../DatabaseXBeeTransfer.cfg");
    if(cfgfile.is_open())
    {
        int i = 0;
        while(getline(cfgfile,line))
        {
            if(line == "end")
            {
                break;
            }
            else
            {
                switch(i)
                {
                case 0:
                    db_name = line;
                    break;
                case 1:
                    db_user = line;
                    break;
                case 2:
                    db_pass = line;
                    break;
                case 3:
                    usb_loc = line;
                    break;
                case 4:
                    dest_address = line;
                    break;
                case 5:
                    pollrate_seconds = line;
                    break;
                case 6:
                    pollrate_nanoseconds = line;
                    break;
                case 7:
                    target_pollrate = line;
                    break;
                case 8:
                    is_isrplane = line;
                    break;
                case 9:
                    gcs_datastream = line;
                    break;
                case 10:
                    gcs_dest_address = line;
                    break;
                case 11:
                    platform_id = line;
                    break;
                default:
                    cout << "Switch Error" << endl;
                }
            }
            if(i==11)
            {
                break;
            }
            i++;
        }


        cfgfile.close();

        // Check to see if any of the config fields are empty
        if(db_name.empty())
        {
            cout << "ERROR: No database name set in config file." << endl;
            bad_config = true;
        }
        if(db_user.empty())
        {
            cout << "ERROR: No username set in config file." << endl;
            bad_config = true;
        }
        if(db_pass.empty())
        {
            cout << "ERROR: No password set in config file." << endl;
            bad_config = true;
        }
        if(usb_loc.empty())
        {
            cout << "ERROR: No USB for the XBee is set in the config file." << endl;
            bad_config = true;
        }
        if(dest_address.empty())
        {
            cout << "ERROR: No address is set for the packet destination in the config file." << endl;
            bad_config = true;
        }
        if(pollrate_seconds.empty())
        {
            cout << "ERROR: No pollrate_seconds  set in config file." << endl;
            bad_config = true;
        }
        if(pollrate_nanoseconds.empty())
        {
            cout << "ERROR: No pollrate_nanoseconds set in config file." << endl;
            bad_config = true;
        }
        if(target_pollrate.empty())
        {
            cout << "ERROR: No target_pollrate set in config file." << endl;
            bad_config = true;
        }
        if(is_isrplane.empty() && (is_isrplane == "y" || is_isrplane == "n"))
        {
            cout << "ERROR: Whether or not plane is isr not defined" << endl;
            bad_config = true;
        }
        if(gcs_datastream.empty() && (gcs_datastream == "y" || gcs_datastream == "n"))
        {
            cout << "ERROR: Whether or not to send data to gcs is not defined" << endl;
            bad_config = true;
        }
        if(gcs_dest_address.empty())
        {
            cout << "ERROR: There is no address destination for the ground control station" << endl;
            bad_config = true;
        }
        if(platform_id.empty())
        {
            cout << "ERROR: There is no platform id configured for this plane" << endl;
            bad_config = true;
        }
    }
    else
    {
        cout << "Unable to open config file for the database transfer." << endl;
        bad_config = true;
    }

    struct timespec tim_con, tim2_con;
    tim_con.tv_sec =  1;
    tim_con.tv_nsec = 9999999;

    UAV_DatabaseConnect(db_name,db_user,db_pass);
    nanosleep(&tim_con,&tim2_con);

    //Archive Old Data
    UAV_flightdatamigration();
    UAV_ClearImageString_R();

    bool xbee_connected = false;
    bool address_connected1 = false;
    bool address_connected2 = false;
    bool address_connected3 = false;
    int dest_id = 0;

    bool is_isrplane_bool = false;
    if(is_isrplane == "y")
        is_isrplane_bool = true;
    bool gcs_datastream_bool = false;
    if(gcs_datastream == "y")
        gcs_datastream_bool = true;

    int platform_id_int = atoi(platform_id.c_str());
    cout << "Platform ID: " << platform_id_int << endl;
    std::condition_variable cond;
    debug::Log::Suppress(comnet::debug::LOG_NOTIFY);
    debug::Log::Suppress(comnet::debug::LOG_WARNING);
    cout << sizeof(comnet::Header) << std::endl;
    Comms comm1(platform_id_int);
    architecture::os::CommMutex mut;
    architecture::os::CommLock commlock(mut);
    comnet::architecture::os::WaitForMilliseconds(commlock, cond, 1000);
    xbee_connected = comm1.InitConnection(ZIGBEE_LINK, usb_loc.c_str(), "", 57600);
    cout << "Init connection succeeded: "
        << boolalpha
        << xbee_connected
        << endl;
    cout << "Plane Address:	'" << dest_address << "'" << endl;
    if(platform_id_int == 3)
    {
        address_connected1 = comm1.AddAddress(6, "0013A2004067E4A0");
        address_connected2 = comm1.AddAddress(2, "0013A2004067E49F");
        dest_id = 2;
    }
    if(gcs_datastream_bool)
    {
        cout << "GCS Address:	'" << gcs_dest_address << "'" << endl;
        if(platform_id_int == 2)
        {
            address_connected1 = comm1.AddAddress(6, "0013A2004067E49F");
            dest_id = 6;
        }
        if(platform_id_int == 6)
        {
            address_connected2 = comm1.AddAddress(2, "0013A2004067E4A0");
            dest_id = 2;
        }
        address_connected3 = comm1.AddAddress(3, "0013A20040917974"); //gcs_dest_address.c_str()
	    address_connected3 = comm1.AddAddress(4, "0013A2004067E4AE"); //Actually GCS
    }
    cout << "--Connected Status--"
        << endl
        << boolalpha
        << "Plane_Linked:	"
        << address_connected1
        << address_connected2
        << endl
        << "GCS_Linked:	"
            << "Dest id "
            << dest_id
        << address_connected3
        << endl;

    //comm1.LinkCallback(new db_GPSData(), new Callback((callback_t) db_GPSDataCallback));
    //comm1.LinkCallback(new db_HEADINGData(), new Callback((callback_t) db_HEADINGDataCallback));
    //comm1.LinkCallback(new db_TARGETData(), new Callback((callback_t) db_TARGETDataCallback));
    //comm1.LinkCallback(new ngcp::VehicleWaypointCommand(), new Callback((callback_t) VehicleWaypointCommandCallback));
    comm1.LinkCallback(new db_mission_status(), new Callback((callback_t) db_mission_statusCallback));
    comm1.LinkCallback(new image_packet(), new Callback((callback_t) image_packetcallback));
    comm1.LinkCallback(new ngcp::VehicleInertialState(), new Callback((callback_t) VehicleInertialStateCallback));
    comm1.LinkCallback(new ngcp::TargetDesignationCommand(), new Callback((callback_t) TargetDesignationCommandCallback));
    comm1.LinkCallback(new ngcp::VehicleGlobalPosition(), new Callback((callback_t) VehicleGlobalPositionCallback));

    comm1.Run();

    // Structs for using nanoseconds and assigning our values from our config file to them.
    struct timespec tim, tim2;
    tim.tv_sec =  atoi(pollrate_seconds.c_str());
    tim.tv_nsec = atoi(pollrate_nanoseconds.c_str());

    //For TARGET pollrate of every X seconds
    double target_pollrate_value = atoi(target_pollrate.c_str());
    double total_pollrate = atoi(pollrate_seconds.c_str()) + (atoi(pollrate_nanoseconds.c_str()) * .000000001);
    int target_pollrate_counter = target_pollrate_value / total_pollrate;
    cout << "t_pollrate = ";
    cout << total_pollrate << endl;
    cout << "target_pollrate_counter = ";
    cout << target_pollrate_counter << endl;


    // Loop to constantly send unpulled data to the other plane.
    // Also checks to see if the data is empty or out of reasonable range.
    int test_count = 10;
    int i=0;
    double a[3] = {};
    double b[3] = {};
    double c[3] = {};
    double d[3] = {};
    int sent_mission_change = 0;
    int temp2_status = 0;
    bool temp_image_flag = false;
    //callback_struct data_stuct();
    if(!bad_config && xbee_connected && (address_connected1 || address_connected2))
    {
        while(true)
        {
            UAV_PullLatestGPS_LOCAL(a);
            UAV_PullLatestHEADING_LOCAL(b);
            UAV_PullTARGET_LOCAL(c);
            ngcp::VehicleInertialState GPSData((uint16_t)platform_id_int,(real32_t)a[2],(real32_t)a[1],(real32_t)a[0],0,0,(real32_t)b[0]);
            ngcp::TargetDesignationCommand TARGETData((uint16_t)platform_id_int,0,0,0,(real32_t)c[2],(real32_t)c[1],(real32_t)c[0]);
            ngcp::VehicleGlobalPosition VehicleGlobalPositionData((uint16_t)platform_id_int,(real32_t)a[2],(real32_t)a[1],(real32_t)a[0],0,0,0);

            if(platform_id_int != 3 && (a[0] != 0 || a[1] != 0 || a[2] != 0))
            {
                comm1.Send(GPSData,dest_id);
            }
            else
            {
                cout << "NO gps data or EMPTY gps data trying to be sent" << endl;
            }
            int temp3_status = UAV_CheckMissionStatus();
            if(temp3_status == 0)
            {
                temp_image_flag = false;
                UAV_ClearImageString_S();
            }

            if(is_isrplane_bool && c[1] != 0 && c[2] != 0)
            {
                if(UAV_ImageStringNum_S() > 0 && !temp_image_flag)
                {
                    for (int k = 1; k <= UAV_ImageStringNum_S(); ++k)
                    {
                        image_packet img_pkt(UAV_PullImageString_S(k));
                        comm1.Send(img_pkt,3);
                        usleep(10000);
                    }
                    temp_image_flag = true;
                }
                comm1.Send(TARGETData,3);
                comm1.Send(TARGETData,4);
            }
            if(gcs_datastream_bool)
            {
                //comm1.Send(VehicleGlobalPositionData,3); //Uncomment when debug gcs
                cout << "Sending: "
                     << "Alt: "
                     << VehicleGlobalPositionData.altitude
                     << " Long: "
                     << VehicleGlobalPositionData.longitude
                     << " Lat: "
                     << VehicleGlobalPositionData.latitude
                     << endl;
                comm1.Send(GPSData,4);
            }
            if(platform_id_int == 3) // COMPUTER
            {
                int temp_status = UAV_CheckMissionStatus();
                cout << "Mission Status: " << temp_status << endl;
                db_mission_status mission_status(temp_status);
                if(temp_status != temp2_status)
                {
                    //comm1.Send(TARGETData,2);
                    if(temp_status == 1)
                    {
                        comm1.Send(mission_status, 2);
                        comm1.Send(mission_status, 6);
                        sent_mission_change++;
                        cout << "GCS SENDING MISSION START STATUS UPDATE" << endl;
                    }
                    else if(temp_status == 2)
                    {
                        comm1.Send(mission_status, 6);
                        sent_mission_change++;
                        cout << "SENT NOT BALL ACKNOWLEDGEMENT OF IMAGE" << endl;
                    }
                    else if(temp_status == 3)
                    {
                        comm1.Send(mission_status, 6);
                        sent_mission_change++;
                        cout << "SENT POSITIVE IDENTIFICATION ACKNOWLEDGEMENT OF IMAGE" << endl;
                    }
                    else if(temp_status == 0)
                    {
                        comm1.Send(mission_status, 2);
                        comm1.Send(mission_status, 6);
                        sent_mission_change++;
                        cout << "SENDING TO RESTART MISSION" << endl;
                    }
                }
                if(sent_mission_change == 3)
                {
                    temp2_status = temp_status;
                    sent_mission_change = 0;
                }
            }
            while(!callback_queue.empty())
            {
                switch(callback_queue.front().index)
                {

                    case 0: // Mission Update
                        UAV_SetMissionStatus(callback_queue.front().data1);
                        break;
                    case 1: // GPS Heading Update
                        if(callback_queue.front().source == ugv)
                        {
                            UAV_InsertGPS_UGV(callback_queue.front().data1, callback_queue.front().data2, callback_queue.front().data3);
                            UAV_InsertHEADING_UGV(callback_queue.front().data4,0,0);
                        }
                        if(callback_queue.front().source == uav_sig || callback_queue.front().source == uav_val)
                        {
                            UAV_InsertGPS_ALT(callback_queue.front().data1, callback_queue.front().data2, callback_queue.front().data3);
                            UAV_InsertHEADING_UGV(callback_queue.front().data4,0,0);
                        }
                        break;
                    case 2: // Target Update
                        UAV_InsertTARGET_LOCAL(callback_queue.front().data1,callback_queue.front().data2,callback_queue.front().data3);
                        break;
                    case 3: // Image Update
                        UAV_InsertImageString_R(callback_queue.front().data5);
                        break;
                    case 4: // Global Position Update
                        UAV_InsertGPS_ALT(callback_queue.front().data1,callback_queue.front().data2,callback_queue.front().data3);
                        break;
                    default:
                        cout << "ERROR: callback_queue could not find the correct index" << endl;
                        break;
                }
                callback_queue.pop();
                //cout << "POPPED QUEUE" << endl;
            }

            nanosleep(&tim,&tim2); // Sleep function
            i++;
        }
          cin.ignore();
    }
    else
        cout << "ERROR: Please make sure your config file is correct and your xbee is connected" << endl;
    return 0;
}


#pragma clang diagnostic pop
