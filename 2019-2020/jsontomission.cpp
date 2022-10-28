/**
 *
 * NGCP UAV - Mission Communication Program
 * Author: Tristan Cady, Student ECE Department
 *
 **/
#include "jsontomission.h"

using json = nlohmann::json;

void jsontomission::fillMission(json missionInfo)
{
    auto taskType = missionInfo["taskType"].get<std::string>();

    // TAKEOFF MISSION
    if(taskType == "takeoff"){
        tkf.takeoff_lat = missionInfo["lat"].get<float>();
        tkf.takeoff_lon = missionInfo["lng"].get<float>();
        tkf.takeoff_alt = missionInfo["alt"].get<float>();
        tkf.loiter_lat = missionInfo["loiter"]["lat"].get<float>();
        tkf.loiter_lon = missionInfo["loiter"]["lng"].get<float>();
        tkf.loiter_alt = missionInfo["loiter"]["alt"].get<float>();
        tkf.loiter_radius = missionInfo["loiter"]["radius"].get<float>();
        tkf.loiter_direction = missionInfo["loiter"]["direction"].get<float>();
    }

    // LOITER MISSION
    else if (taskType == "loiter") {
        ltr.loiter_lat = missionInfo["lat"].get<float>();
        ltr.loiter_lon = missionInfo["lng"].get<float>();
        ltr.loiter_alt = missionInfo["alt"].get<float>();
        ltr.loiter_radius = missionInfo["radius"].get<float>();
        ltr.loiter_direction = missionInfo["direction"].get<float>();

    }

    // ISR SEARCH MISSION
    else if (taskType == "isrSearch") {
        isr.search_alt = missionInfo["alt"].get<float>();
        isr.topleft_lat = missionInfo["waypoints"][0]["lat"].get<float>();
        isr.topleft_lon = missionInfo["waypoints"][0]["lng"].get<float>();
        isr.botleft_lat = missionInfo["waypoints"][1]["lat"].get<float>();
        isr.botleft_lon = missionInfo["waypoints"][1]["lng"].get<float>();
        isr.botright_lat = missionInfo["waypoints"][2]["lat"].get<float>();
        isr.botright_lon = missionInfo["waypoints"][2]["lng"].get<float>();
    }

    // PAYLOAD DROP MISSION
    else if (taskType == "payloadDrop") {
        pid.first_lat = missionInfo["waypoints"][0]["lat"].get<float>();
        pid.first_lon = missionInfo["waypoints"][0]["lng"].get<float>();
        pid.first_alt = missionInfo["waypoints"][0]["alt"].get<float>();
        pid.second_lat = missionInfo["waypoints"][1]["lat"].get<float>();
        pid.second_lon = missionInfo["waypoints"][1]["lng"].get<float>();
        pid.second_alt = missionInfo["waypoints"][1]["alt"].get<float>();
    }

    // LAND MISSION
    else if (taskType == "land") {
        lnd.first_lat = missionInfo["waypoints"][0]["lat"].get<float>();
        lnd.first_lon = missionInfo["waypoints"][0]["lng"].get<float>();
        lnd.first_alt = missionInfo["waypoints"][0]["alt"].get<float>();
        lnd.second_lat = missionInfo["waypoints"][1]["lat"].get<float>();
        lnd.second_lon = missionInfo["waypoints"][1]["lng"].get<float>();
        lnd.second_alt = missionInfo["waypoints"][1]["alt"].get<float>();
    }

    else if (taskType == "waiting") {
        printf("Waiting mission: Nothing to fill\n");
    }

    // Task type not implemented
    else {
        printf("ERROR: Could not fill mission because tasktype does not exist\n");
    }
}
