/**
 *
 * NGCP UAV - Mission Communication Program
 * Author: Tristan Cady, Student ECE Department
 *
 **/

/**STANDARD INCLUDES**/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <zconf.h>
#include <ctime>
#include <signal.h>

/**CUSTOM INCLUDES**/
#include "UAV_Database.h"
#include "comm_xbee.h"
#include "comm_packet.h"
#include <json.hpp>
#include "jsontomission.h"


/**
 *  MAC ADDRESS TEST
 *  00-13-A2-00-41-94-78-3A
 *
**/

using json = nlohmann::json;


/** GLOBAL VARIABLES **/
bool f_connectionAck = false;
bool f_start = false;
bool f_addMission = false;
bool f_stop = false;
int frame_id = 0;
int start_frame_id = 0;
int addMission_frame_id = 0;
int stop_frame_id = 0;
int target_confirmation=0;
UAV_Database db;
jsontomission mission;
volatile sig_atomic_t stop;
std::string status;

void inthand(int signum) {
    stop = 1;
}

void gcsCallback(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data)
{
    if ((*pkt)->dataLen > 0)
    {
        std::vector<uint8_t> packet;
        for (int i = 0; i < (*pkt)->dataLen; i++) {
            packet.push_back((*pkt)->data[i]);
        }
        json packet_rec = json::from_msgpack(packet);
        printf("%s\n",packet_rec.dump().c_str());
        auto m_type = packet_rec["type"].get<std::string>();
        auto sid = packet_rec["sid"].get<int>();
        frame_id = packet_rec["id"].get<int>();
        //printf("FRAME ID: %i\n", frame_id);
        printf("Message Type: %s\n", m_type.c_str());
        if(sid == 0) {
            if (m_type == "connectionAck") {
                f_connectionAck = true;
            } else if (m_type == "start") {
                f_start = true;
                status = "waiting";
                start_frame_id = packet_rec["id"].get<int>();
            } else if (m_type == "addMission") {
                f_addMission = true;
                auto taskType = packet_rec["missionInfo"]["taskType"].get<std::string>();
                printf("Task Type: %s\n", taskType.c_str());
                addMission_frame_id = packet_rec["id"].get<int>();

                json missionInfo = packet_rec["missionInfo"];


                // IDLE OR WAITING
                if (taskType == "waiting") {
                    db.SetMissionStatus(0);
                    status = "ready";
                }

                // TAKEOFF MISSION
                else if(taskType == "takeoff"){
                    db.InsertMission(missionInfo.dump());
                    db.SetMissionStatus(1);
                    status = "running";
                }

                // LOITER MISSION
                else if (taskType == "loiter") {
                    db.InsertMission(missionInfo.dump());
                    db.SetMissionStatus(2);
                    status = "running";
                }

                // ISR SEARCH MISSION
                else if (taskType == "isrSearch") {
                    db.InsertMission(missionInfo.dump());
                    db.SetMissionStatus(3);
                    status = "running";
                }

                // PAYLOAD DROP MISSION
                else if (taskType == "payloadDrop") {
                    db.InsertMission(missionInfo.dump());
                    db.SetMissionStatus(4);
                    status = "running";
                }

                // LAND MISSION
                else if (taskType == "land") {
                    db.InsertMission(missionInfo.dump());
                    db.SetMissionStatus(5);
                    status = "running";
                }

                // Task type not implemented
                else {
                    printf("Task type not defined in logic loop\n");
                }

            } else if (m_type == "pause") {
                status = "pause";
            } else if (m_type == "resume") {
                status = "running";
            } else if (m_type == "stop") {
                f_stop = true;
                stop_frame_id = packet_rec["id"].get<int>();
                status = "ready";
            } else if (m_type == "ack") {

            } else if (m_type == "target_confirmation") {
                target_confirmation = packet_rec["confirm"].get<int>();
                db.SetTargetStatus(target_confirmation);
            }else {
                printf("Received message with unknown type\n");
            }
        }
    }
    else
        printf("Empty rx message");
}

int main(int argc, char** argv)
{
    //Database initialization
    db.Connect("plane1","root","ngcp");
    std::cout << "Flight Id:" << db.flightdatamigration() << std::endl;
    status = "ready";
    comm_xbee xbee;
    comm_packet c_payload(100,0);

    //0013A200419477B6
    std::string gcs_address = "0013A2004194754E";
    std::string mini_gcs_address = "0013A2004194783A";
    xbee.initialize("/dev/ttyUSB1",57600);
    xbee.addAddress(0,gcs_address);
    xbee.addAddress(1,mini_gcs_address);
    xbee.callbackSet(0, gcsCallback);

    usleep(1000000);
    xbee.send(0,c_payload.connect_payload());

    signal(SIGINT, inthand);

    double pos[3];
    double attitude[3];
    double poi[3];
    int cur_poi_id = -1;
    int pas_poi_id = -1;
    int target_status;
    db.SetMissionStatus(0);

    while(!stop)
    {

        //Acknowledgement loop
        if (f_start){
            xbee.send(0,c_payload.ack(start_frame_id));
            f_start = false;
        } else if (f_addMission) {
            xbee.send(0, c_payload.ack(addMission_frame_id));
            f_addMission = false;
        } else if (f_stop) {
            xbee.send(0, c_payload.ack(stop_frame_id));
            f_stop = false;
        }

        //Plane position and status update loop
        db.PullLatestGPS_LOCAL(pos);
        db.PullLatestHEADING_LOCAL(attitude);
        xbee.send(0,c_payload.update((float)pos[1],(float)pos[2],(float)pos[0],(float)attitude[0],0.0,status));

        //Point of interest checking
        db.PullTARGET_LOCAL(poi,&cur_poi_id);
        if(cur_poi_id > pas_poi_id){
            pas_poi_id = cur_poi_id;
            xbee.send(0,c_payload.poi((float)poi[1],(float)poi[2]));
        }

        //Image detected send loop
        target_status = db.CheckTargetStatus();
        if(target_status == 1) {
            for (int k = 1; k <= db.ImageStringNum_S(); ++k) {
                xbee.send(1, c_payload.image_string(db.PullImageString_S(k)));
                usleep(10000);
            }
            db.SetTargetStatus(2);
        }
        usleep(750000);
    }

    printf("CLOSING PROGRAM........\n");
    xbee.close(0);
    xbee.shutdown();
    db.DatabaseDisconnect();
}
