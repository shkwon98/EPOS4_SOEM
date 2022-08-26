#include "CTcpPacket.hpp"

CTcpPacket::CTcpPacket()
{
    memset(&server_addr, 0, sizeof(server_addr));
    memset(&client_addr, 0, sizeof(client_addr));

    if ((server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1)
    {
        perror("TCP: socket() error");
        exit(1);
    }

    int option = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(TCP_PORT);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("TCP: bind() error");
        exit(1);
    }

    if (listen(server_fd, 5) < 0)
    {
        perror("TCP: listen() error");
        exit(1);
    }

    if ((client_fd = accept(server_fd, (struct sockaddr *)&client_addr, (socklen_t *)&client_addr_size)) < 0)
    {
        perror("TCP: accept() error");
        exit(1);
    }

    int flag = fcntl(client_fd, F_GETFL, 0);
    fcntl(client_fd, F_SETFL, flag | O_NONBLOCK);
}

CTcpPacket::~CTcpPacket()
{
    close(server_fd);
}


int CTcpPacket::readPacket()
{
    retval = read(client_fd, rxBuffer, sizeof(rxBuffer));
    if (retval < 0)
    {
        if (errno != EAGAIN)
        {
            // perror("\nTCP: read() error");
            // printf("----- Socket close -----\n");
            exit(1);
        }
    }
    else if (retval == 0)
    {
        // perror("\n----- Socket close -----");
        isRemoteON = false;
    }
    else    // Packet Process
    {
        for (int i = 0; i < RX_BUFFER_SIZE; i++)
        {
            if (rxBuffer[i] == 13 && rxBuffer[i + 1] == 10)
            {
                memcpy(&dataSize, &rxBuffer[i + 2], sizeof(uint16_t));
                memcpy(packetBuffer, &rxBuffer[i + 4], dataSize);
                break;
            }
        }

        header = 0;
        decodeIndex = 0;

        memcpy(&header, &packetBuffer[0], sizeof(short));
        decodeIndex += sizeof(uint16_t);
    }
    return retval;
}