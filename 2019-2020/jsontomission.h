/**
 *
 * NGCP UAV - Mission Communication Program
 * Author: Tristan Cady, Student ECE Department
 *
 **/
#ifndef ISR_MAIN_JSONTOMISSION_H
#define ISR_MAIN_JSONTOMISSION_H

#include <json.hpp>

using json = nlohmann::json;

class jsontomission {
public:
    struct takeoff{
       float takeoff_lat;
       float takeoff_lon;
       float takeoff_alt;
       float loiter_lat;
       float loiter_lon;
       float loiter_alt;
       float loiter_radius;
       float loiter_direction;
    };
    struct loiter{
        float loiter_lat;
        float loiter_lon;
        float loiter_alt;
        float loiter_radius;
        float loiter_direction;
    };
    struct isrSearch{
        float search_alt;
        float topleft_lat;
        float topleft_lon;
        float botleft_lat;
        float botleft_lon;
        float botright_lat;
        float botright_lon;
    };
    struct payloadDrop{
        float first_lat;
        float first_lon;
        float first_alt;
        float second_lat;
        float second_lon;
        float second_alt;
    };
    struct land{
        float first_lat;
        float first_lon;
        float first_alt;
        float second_lat;
        float second_lon;
        float second_alt;
    };

    // Struct implementations
    takeoff tkf;
    loiter ltr;
    isrSearch isr;
    payloadDrop pid;
    land lnd;

    void fillMission(json missionInfo);
};


#endif //ISR_MAIN_JSONTOMISSION_H
