#include "udpPacket.h"


UDP_Packet::UDP_Packet()
{
    memset(&server_addr, 0, sizeof(server_addr));

    if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
        errorHandling("UDP: socket() error");

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(GUI_PC_IP);
    server_addr.sin_port = htons(UDP_PORT);
}
UDP_Packet::~UDP_Packet()
{
    close(socket_fd);
}

void UDP_Packet::setCommandHeader(uint16_t header)
{
   // Initialize Variable
    encodeIndex = 0;

    for (int i = 0; i < TX_BUFFER_SIZE; i++)
    {
        txBuffer[i] = 0;
    }

    // Packet Header 1
    txBuffer[encodeIndex] = 13;
    encodeIndex++;

    // Packet Header 2
    txBuffer[encodeIndex] = 10;
    encodeIndex++;

    // Command Header
    memcpy(&txBuffer[encodeIndex], &header, sizeof(header));
    encodeIndex = encodeIndex + sizeof(header);
}


void UDP_Packet::sendPacket()
{
   ///////////////////////////////////////////////////////
   ///////////////////////////////////////////////////////
   // Packet Header 1,
   // Packet Header 2,
   // Command Header,
   // Data
   ///////////////////////////////////////////////////////
   ///////////////////////////////////////////////////////

    sendto(socket_fd, txBuffer, encodeIndex, 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
}
