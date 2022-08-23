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
            thread th_ecatCheck(SOEM::ecatCheck);         // Start SOEM ecatCheck Thread

            motorTask.rtLoopStart(CONTROL_PERIOD_IN_ns);  // Start Real-Time Control Thread

            thread th_tcpCommand(&tcpCommand);            // Start TCP Command Receiver Thread
            th_tcpCommand.join();                         // Block until Remote Program Shutdown
            // sleep(60);

            cout << "\n\nRemote Program is off. Motor Power off.\n\n";
            SOEM::inOP = false;      // Start Program End Process
            sleep(1);                // Wait for the motor power to turn off
            motorTask.rtLoopStop();  // Stop and Join Real-Time Control Thread
            th_ecatCheck.join();     // Join SOEM ecatCheck Thread 
        }

        SOEM::terminateEtherCAT();
    }
    else
    {
        cout << "Usage: main [ifname]\n[ifname] = eth0 for example\n\n";
    }

    cout << "End program\n";
    return 0;
}
