#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <math.h>
#include <iostream>

#include "ecatErrorHandle.hpp"
#include "scheduling.hpp"
#include "CPdoMapping.hpp"
#include "CTcpPacket.hpp"
#include "socketDef.hpp"
#include "CEcatCommand.hpp"
#include "CControlThread.hpp"
// #include "mytest.h"

using namespace ecat;


#define NUMOF_EPOS4 1

static OSAL_THREAD_HANDLE thread0;
static OSAL_THREAD_HANDLE thread1;
static OSAL_THREAD_HANDLE thread2;

volatile int expectedWKC;
volatile bool inOP = false;

SERVO_IO_pt EPOS4[NUMOF_EPOS4];
bool bRunStart = false;
short mode;
INPUT_LIST input;

pthread_mutex_t mtx_IOMap;


OSAL_THREAD_FUNC tcpCommunicate()
{
    CTcpPacket* tcpPacket;
    tcpPacket = new CTcpPacket();

    while (1)  // 10ms non-rt loop
    {
        if (inOP && tcpPacket->readPacket() > 0)
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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char *argv[])
{
    std::cout << "SOEM (Simple Open EtherCAT Master)\n< EPOS4 Motor Controller >\n\n";

    if (argc > 1)
    {
        osal_thread_create(&thread0, 128000, (void*)&ecatCheck, NULL);
        osal_thread_create(&thread2, 128000, (void*)&tcpCommunicate, NULL);

        CEcatMaster* master;
        master = new CEcatMaster(argv[1]);

        master->printState1();

        for (int slave = 1; slave <= ec_slavecount; ++slave)
        {
            master->setupPDO(slave, CPdoMapping::mapMotorPDOs);
        }
        master->configMap();
        master->configDC();
        master->movetoState(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

        master->map_structs(EPOS4, NUMOF_EPOS4);
        expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        // std::cout << "Calculated workcounter " << expectedWKC << std::endl;

        master->sendAndReceive(EC_TIMEOUTRET);
        master->config_ec_sync0(1, TRUE, CONTROL_PERIOD_IN_ns, 0); // SYNC0 on slave 1

        master->movetoState(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);
        if (ec_slave[0].state == EC_STATE_OPERATIONAL)
        {
            std::cout << "Operational state reached for all slaves!\n\n";
            inOP = true; // activate cyclic process

            pthread_mutex_init(&mtx_IOMap, NULL);
            osal_thread_create_rt(&thread1, 204800, (void*)&sync_thread, NULL);

            CControlThread* epos4;
            epos4 = new CControlThread();

            epos4->rtLoopStart(CONTROL_PERIOD_IN_ns);

            while (1)
            {
                sleep(1);
            }

            inOP = false;
            delete epos4;
        }
        else
        {
            std::cout << "Not all slaves reached operational state.\n";

            master->printState2();
            for (int i = 0; i < NUMOF_EPOS4; i++)
            {
                master->config_ec_sync0(i + 1, FALSE, 0, 0);
            }
        }

        master->movetoState(0, EC_STATE_INIT, EC_TIMEOUTSTATE);
        master->close_master();
        delete master;
    }

    else
    {
        ec_adaptert *adapter = NULL;
        std::cout << "Usage: main ifname\nifname = eth0 for example\n\nAvailable adapters:\n";
        adapter = ec_find_adapters();
        while (adapter != NULL)
        {
            std::cout << "    - " << adapter->name << "  (" << adapter->desc << ")\n";
            adapter = adapter->next;
        }
        ec_free_adapters(adapter);
    }
    std::cout << "End program\n";

    return (0);
}