/**
 *
 * NGCP UAV - Mission Communication Program
 * Author: Tristan Cady, Student ECE Department
 *
 **/

#include "comm_xbee.h"

void comm_xbee::initialize(std::string port, int baudrate)
{
    printf("Initializing XBee on port %s with baudrate %i\n",port.c_str(),baudrate);
    if ((ret = xbee_setup(&xbee, "xbee3", port.c_str() , 57600)) != XBEE_ENONE) {
        printf("ERROR: ret: %d (%s)\n", ret, xbee_errorToStr(ret));
    }

}

void comm_xbee::addAddress(int nodeid, std::string address)
{
    if(nodeid >= 0 && nodeid <= 9)
    {
        xbee_conAddress addr = toXbeeAddress(address);
        if ((ret = xbee_conNew(xbee, &con[nodeid], "Data", &addr)) != XBEE_ENONE)
        {
            printf("ERROR: xbee_conNew() returned: %d (%s)\n", ret, xbee_errorToStr(ret));
        }
    }
    else
    {
        printf("ERROR: Xbee node out of range (0-9)\n");
    }
}

void comm_xbee::send(int nodeid, std::vector<uint8_t> msg)
{
    if(msg.size() > 244)
    {
        printf("ERROR: Message size too large. Limit is > 244 bytes");
    }
    else
    {
        //ret = xbee_connTx(con[nodeid], &txRet, test, sizeof(test));
        ret = xbee_connTx(con[nodeid], &txRet, msg.data(), msg.size());
        int bytes = sizeof(msg[0]) * msg.size();
        if(ret == 0)
            printf("Sending Message with byte size: %i \n", bytes);
        else
            printf("ERROR: Could not send message: %d\n",ret);
    }
}

void comm_xbee::callbackSet(int nodeid, xbee_t_conCallback callback_function)
{
    if ((ret = xbee_conCallbackSet(con[nodeid], callback_function, nullptr)) != XBEE_ENONE) {
        printf("xbee_conCallbackSet() returned: %d", ret);
    }
}

void comm_xbee::close(int nodeid)
{
    if ((ret = xbee_conEnd(con[nodeid])) != XBEE_ENONE) {
        printf("ERROR: xbee_conEnd() returned: %d", ret);
    }
}

void comm_xbee::shutdown()
{
    xbee_shutdown(xbee);
}

xbee_conAddress comm_xbee::toXbeeAddress(const std::string &mac_addr)
{
    struct xbee_conAddress address{};
    memset(&address, 0, sizeof(address));
    address.addr64_enabled = 1;
    int addr_count = 0;
    for (int i = 0; i < 16; i = i + 2)
    {
        std::string sub_addr1 = mac_addr.substr(i,1);
        std::string sub_addr2 = mac_addr.substr(i+1,1);
        address.addr64[addr_count] =
                static_cast<unsigned char>(std::strtol(sub_addr1.c_str(), nullptr, 16) * 16 +
                std::strtol(sub_addr2.c_str(), nullptr, 16));
        addr_count++;
    }
    return address;

}

comm_xbee::comm_xbee() = default;
