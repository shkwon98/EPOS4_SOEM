#ifndef UDPPACKET_H_
#define UDPPACKET_H_

#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>

#include "errorHandling.h"
#include "Macro.h"

#define UDP_PORT 3000

class UDP_Packet
{
public:
    UDP_Packet();
    ~UDP_Packet();

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

#endif /* UDPPACKET_H_ */
