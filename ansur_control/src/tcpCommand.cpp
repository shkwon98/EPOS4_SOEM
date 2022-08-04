#include "tcpCommand.hpp"

bool bRunStart = false;
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
                case COMMAND_SET_TASK_PARAM:
                    tcpPacket->decode(input.taskParam);
                    break;

                case COMMAND_RUN_CSP:
                    tcpPacket->decode(bRunStart);
                    break;

                case COMMAND_RUN_CSV:
                    tcpPacket->decode(bRunStart);
                    tcpPacket->decode(input.velocity);
                    break;

                case COMMAND_RUN_CST:
                    tcpPacket->decode(bRunStart);
                    tcpPacket->decode(input.torque);
                    break;
            }
        }
        osal_usleep(10000);
    }

    delete tcpPacket;
}