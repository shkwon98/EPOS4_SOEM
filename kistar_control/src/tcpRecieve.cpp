#include "tcpRecieve.hpp"

short mode;
JOINT_INPUT_LIST joint_input;
MOTION_INPUT_LIST motion_input_1, motion_input_2, motion_input_3, motion_input_4;

/** Thread function that will be responsible for receiving tcp packets.
 *
*/
void tcpRecieve()
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
                    tcpPacket->decode(joint_input);
                    break;

                case MODE_MOTION:
                    tcpPacket->decode(motion_input_1);
                    tcpPacket->decode(motion_input_2);
                    tcpPacket->decode(motion_input_3);
                    tcpPacket->decode(motion_input_4);
                    break;

                default:
                    break;
            }
        }
        osal_usleep(1000);
    }

    delete tcpPacket;
}
