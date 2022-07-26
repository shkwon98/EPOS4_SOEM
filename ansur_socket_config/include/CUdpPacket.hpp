#ifndef CUDPPACKET_HPP_
#define CUDPPACKET_HPP_

#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#include "socketDef.hpp"

#define UDP_PORT 3000

class CUdpPacket
{
public:
    CUdpPacket();
    ~CUdpPacket();

    void setCommandHeader(uint16_t header);

    template <class T>
    void encode(T &val)
    {
        memcpy(&txBuffer[encodeIndex], &val, sizeof(val));
        encodeIndex = encodeIndex + sizeof(val);
    }

    void sendPacket();


private:
    struct sockaddr_in server_addr, client_addr;
    int socket_fd;

    int encodeIndex;
    unsigned char txBuffer[TX_BUFFER_SIZE];
};

#endif  // CUDPPACKET_HPP_
