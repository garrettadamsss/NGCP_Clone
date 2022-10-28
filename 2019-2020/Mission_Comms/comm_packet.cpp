/**
 *
 * NGCP UAV - Mission Communication Program
 * Author: Tristan Cady, Student ECE Department
 *
 **/

#include "comm_packet.h"

using json = nlohmann::json;

std::vector<uint8_t> comm_packet::connect_payload()
{
    json packet;
    packet["type"] = "connect";
    packet["id"] = frame_id;
    packet["sid"] = source_id;
    packet["tid"] = target_id;
    packet["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    packet["jobsAvailable"] = {"payloadDrop"};
    std::vector<uint8_t> packet_msgpack = json::to_msgpack(packet);
    frame_id++;
    return packet_msgpack;
}

std::vector<uint8_t> comm_packet::connect_isr()
{
    json packet;
    packet["type"] = "connect";
    packet["id"] = frame_id;
    packet["sid"] = source_id;
    packet["tid"] = target_id;
    packet["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    packet["jobsAvailable"] = {"isrSearch"};
    std::vector<uint8_t> packet_msgpack = json::to_msgpack(packet);
    frame_id++;
    return packet_msgpack;
}

std::vector<uint8_t> comm_packet::update(
        float latitude,
        float longitude,
        float altitude,
        float heading,
        float battery,
        std::string status)
{
    json packet;
    packet["type"] = "update";
    packet["id"] = frame_id;
    packet["sid"] = source_id;
    packet["tid"] = target_id;
    packet["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    packet["lat"] = latitude;
    packet["lng"] = longitude;
    packet["alt"] = altitude;
    packet["heading"] = heading;
    packet["battery"] = battery;
    packet["status"] = status;
    std::vector<uint8_t> packet_msgpack = json::to_msgpack(packet);
    frame_id++;
    return packet_msgpack;
}

std::vector<uint8_t> comm_packet::ack(
        int ack_id)
{
    json packet;
    packet["type"] = "ack";
    packet["id"] = frame_id;
    packet["sid"] = source_id;
    packet["tid"] = target_id;
    packet["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    packet["ackid"]= ack_id;
    std::vector<uint8_t> packet_msgpack = json::to_msgpack(packet);
    frame_id++;
    return packet_msgpack;
}

std::vector<uint8_t> comm_packet::poi(
        float latitude,
        float longitude)
{
    json packet;
    packet["type"] = "poi";
    packet["id"] = frame_id;
    packet["sid"] = source_id;
    packet["tid"] = target_id;
    packet["time"] = std::time(nullptr);
    packet["lat"] = latitude;
    packet["lng"] = longitude;
    std::vector<uint8_t> packet_msgpack = json::to_msgpack(packet);
    frame_id++;
    return packet_msgpack;
}

std::vector<uint8_t> comm_packet::complete()
{
    json packet;
    packet["type"] = "complete";
    packet["id"] = frame_id;
    packet["sid"] = source_id;
    packet["tid"] = target_id;
    packet["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::vector<uint8_t> packet_msgpack = json::to_msgpack(packet);
    frame_id++;
    return packet_msgpack;
}

std::vector<uint8_t> comm_packet::custom(std::string msg)
{
    auto packet = json::parse(msg);
    //std::cout << "CUSTOM MESSAGE " << msg << std::endl;
    std::vector<uint8_t> packet_msgpack = json::to_msgpack(packet);
    frame_id++;
    return packet_msgpack;
}

std::vector<uint8_t> comm_packet::target_confirmation(int confirm)
{
    json packet;
    packet["type"] = "target_confirmation";
    packet["confirm"] = confirm;
    std::vector<uint8_t> packet_msgpack = json::to_msgpack(packet);
    frame_id++;
    return packet_msgpack;
}

std::vector<uint8_t> comm_packet::image_string(std::string image)
{
    json packet;
    packet["type"] = "image_string";
    packet["image"] = image;
    std::vector<uint8_t> packet_msgpack = json::to_msgpack(packet);
    frame_id++;
    return packet_msgpack;
}

comm_packet::comm_packet(
        int sid,
        int tid)
{
    frame_id = 1;
    source_id = sid;
    target_id = tid;
}