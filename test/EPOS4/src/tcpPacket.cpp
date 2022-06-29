#include "tcpPacket.h"


TCP_Packet::TCP_Packet()
{
    memset(&server_addr, 0, sizeof(server_addr));
    memset(&client_addr, 0, sizeof(client_addr));

    if ((server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1)
        errorHandling("TCP: socket() error");

    int option = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(TCP_PORT);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        errorHandling("TCP: bind() error");

    if (listen(server_fd, 5) < 0)
        errorHandling("TCP: listen() error");

    if ((client_fd = accept(server_fd, (struct sockaddr *)&client_addr, (socklen_t *)&client_addr_size)) < 0)
        errorHandling("TCP: accept() error");

    int flag = fcntl(client_fd, F_GETFL, 0);
    fcntl(client_fd, F_SETFL, flag | O_NONBLOCK);
}

TCP_Packet::~TCP_Packet()
{
    close(server_fd);
}


int TCP_Packet::readPacket()
{
    retval = read(client_fd, rxBuffer, sizeof(rxBuffer));
    if (retval < 0)
    {
        if (errno != EAGAIN)
        {
            printf("\n\nTCP: read() error\n");
            errorHandling("----- Socket close -----");
        }
    }
    else if (retval == 0)
    {
        errorHandling("\n\n----- Socket close -----");
    }
    else
    {
        for (int i = 0; i < RX_BUFFER_SIZE; i++)
        {
            if (rxBuffer[i] == 13 && rxBuffer[i + 1] == 10)
            {
                memcpy(packetBuffer, &rxBuffer[i + 2], sizeof(packetBuffer));
                break;
            }
        }

        header = 0;
        decodeIndex = 2;

        memcpy(&header, &packetBuffer[0], sizeof(short));
    }
    return retval;
}