#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <math.h>
#include <iostream>

#include "SOEM.hpp"
#include "CPdoMapping.hpp"
#include "CControlThread.hpp"
#include "tcpCommand.hpp"
// #include "mytest.h"

SERVO_IO::SERVO_READ  *EPOS4_READ[EPOS4_NUM];
SERVO_IO::SERVO_WRITE *EPOS4_WRITE[EPOS4_NUM];

static const char* ifname;
static OSAL_THREAD_HANDLE th_ecatCheck;
static OSAL_THREAD_HANDLE th_ecatSync;
static OSAL_THREAD_HANDLE thread0;

pthread_mutex_t mutex;


int main(int argc, char *argv[])
{
    std::cout << "SOEM (Simple Open EtherCAT Master)\n< EPOS4 Motor Controller >\n\n";

    if (argc > 1)
    {
        ifname = argv[1];

        osal_thread_create(&th_ecatCheck, 128000, (void*)&SOEM::ecatCheck, NULL);
        osal_thread_create(&thread0, 128000, (void*)&tcpCommand, NULL);

        SOEM::initializeEtherCAT(ifname);
        SOEM::goingSafeOP(CPdoMapping::mapMotorPDOs);
        SOEM::goingOperational();

        if (SOEM::inOP == true)
        {
            SOEM::mapIOStructs(EPOS4_WRITE, EPOS4_READ, EPOS4_NUM);

            pthread_mutex_init(&mutex, NULL);
            osal_thread_create_rt(&th_ecatSync, 204800, (void*)&SOEM::ecatSync, NULL);


            CControlThread* epos4;
            epos4 = new CControlThread();
            epos4->rtLoopStart(CONTROL_PERIOD_IN_ns);


            while (1) { sleep(1); }

            SOEM::inOP = false;
            delete epos4;
        }

        SOEM::terminateEtherCAT();
    }

    else
    {
        std::cout << "Usage: main ifname\nifname = eth0 for example\n";
    }

    std::cout << "End program\n";
    return 0;
}