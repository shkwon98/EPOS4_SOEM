#include "tcpCommand.hpp"

short mode;
INPUT_LIST input;

void tcpCommand()
{
    CTcpPacket* tcpPacket;
    tcpPacket = new CTcpPacket();

    while (tcpPacket->isRemoteON == true)  // 10ms non-rt loop
    {
        if (tcpPacket->readPacket() > 0)
        {
            mode = tcpPacket->getHeader();
            switch (mode)
            {
                case MODE_OFF:
                    break;

                case MODE_JOINT_POSITION:
                    tcpPacket->decode(input);
                    break;

                case MODE_MOTION:
                    // tcpPacket->decode(input);
                    break;

                default:
                    break;
            }
        }
        osal_usleep(10000);
    }

    delete tcpPacket;
}
