/**
 *
 * NGCP UAV - Mission Communication Program
 * Author: Tristan Cady, Student ECE Department
 *
 **/

#ifndef ISR_MAIN_COMM_PACKET_H
#define ISR_MAIN_COMM_PACKET_H

/**STANDARD INCLUDES**/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <zconf.h>
#include <ctime>
#include <vector>
#include <iostream>
#include <iomanip>
#include <chrono>

/**CUSTOM INCLUDES**/
#include <json.hpp>

class comm_packet {
private:

public:
    int frame_id;
    int source_id;
    int target_id;
    std::vector<uint8_t> connect_payload();
    std::vector<uint8_t> connect_isr();
    std::vector<uint8_t> update(
            float latitude,
            float longitude,
            float altitude,
            float heading,
            float battery,
            std::string status);
    std::vector<uint8_t> ack(
            int ack_id);
    std::vector<uint8_t> complete();
    std::vector<uint8_t> poi(
            float latitude,
            float longitude);
    std::vector<uint8_t> target_confirmation(
            int confirm);
    std::vector<uint8_t> image_string(
            std::string image);
    std::vector<uint8_t> custom(
            std::string msg
            );
    comm_packet(
            int sid,
            int tid);
};


#endif //ISR_MAIN_COMM_PACKET_H
