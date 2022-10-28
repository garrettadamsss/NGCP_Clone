/**
 *
 * NGCP UAV - Mission Communication Program
 * Author: Tristan Cady, Student ECE Department
 *
 **/

#ifndef ISR_MAIN_COMM_XBEE_H
#define ISR_MAIN_COMM_XBEE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <zconf.h>
#include "xbee.h"
#include <vector>

class comm_xbee
{
private:
    struct xbee *xbee;
    struct xbee_con *con[10];
    xbee_err ret;
    unsigned char txRet;

    xbee_conAddress toXbeeAddress(const std::string &mac_addr);

public:
    comm_xbee();

    void initialize(std::string port, int baudrate);

    void addAddress(int nodeid, std::string address);

    void send(int nodeid, std::vector<uint8_t> msg);

    void callbackSet(int nodeid, xbee_t_conCallback callback_function);

    void close(int nodeid);

    void shutdown();

    //JSON STUFF
};

#endif //ISR_MAIN_COMM_XBEE_H
