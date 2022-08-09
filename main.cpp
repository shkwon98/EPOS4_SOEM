#include <iostream>
#include <thread>
#include <mutex>
#include "SOEM.hpp"
#include "CPdoMapping.hpp"
#include "CControlThread.hpp"
#include "tcpCommand.hpp"
// #include "mytest.h"  // This is for vision part.

using namespace std;

static CControlThread motorTask;
PDO_STRUCT EPOS4[EPOS4_NUM];

mutex mtx;


int main(int argc, char *argv[])
{
    cout << "SOEM (Simple Open EtherCAT Master)\n< EPOS4 Motor Controller >\n\n";

    if (argc > 1)
    {
        const char* ifname = argv[1];

        SOEM::initializeEtherCAT(ifname);
        SOEM::goingSafeOP(CPdoMapping::mapMotorPDOs);
        SOEM::goingOperational();
        SOEM::mapIOStructs(EPOS4, EPOS4_NUM);

        if (SOEM::inOP == true)
        {
            // thread th_ecatCheck(SOEM::ecatCheck);

            // Start Real-Time Control Thread
            motorTask.rtLoopStart(CONTROL_PERIOD_IN_ns);
            // motorTask.rtLoopStart(1.3 * CONTROL_PERIOD_IN_ns);

            // Start TCP Command Receiver Thread & Wait for Remote Program Shutdown
            // thread th_tcpCommand(&tcpCommand);
            // th_tcpCommand.join();
            sleep(60);

            // Program End Process
            cout << "\n\nRemote Program is off. Motor Power off.\n\n";
            SOEM::inOP = false;
            sleep(1);  // Wait until motor power is off
            motorTask.rtLoopStop();
            // th_ecatCheck.join();
        }

        SOEM::terminateEtherCAT();
    }
    else { cout << "Usage: main [ifname]\n[ifname] = eth0 for example\n\n"; }

    cout << "End program\n";
    return 0;
}
