#include <iostream>
#include <signal.h>
#include <thread>
#include <mutex>
#include "SOEM.hpp"
#include "CPdoMapping.hpp"
#include "CControlThread.hpp"
#include "tcpRecieve.hpp"

using namespace std;

static CControlThread th_rtTask;
PDO_STRUCT KISTAR[KISTAR_NUM];
mutex mtx;

void interruptHandler(int sig)
{
    signal(sig, SIG_IGN);
    SOEM::inOP = false;
}

int main(int argc, char *argv[])
{
    cout << "SOEM (Simple Open EtherCAT Master)\n< EPOS4 Motor Controller >\n\n";

    if (argc <= 1) cout << "Usage: main [ifname]\n[ifname] = eth0 for example\n\n";
    else
    {
        const char* ifname = argv[1];

        SOEM::initializeEtherCAT(ifname);
        // SOEM::pdoMapping(CPdoMapping::mapSlavePDOs);
        SOEM::goingOperational();
        SOEM::mapIOStructs(KISTAR, KISTAR_NUM);

        if (SOEM::inOP == true)
        {
            thread th_ecatCheck(SOEM::ecatCheck);         // Start SOEM ecatCheck Thread
            th_rtTask.rtLoopStart(CONTROL_PERIOD_IN_ns);  // Start Real-Time Control Thread

#ifdef USE_GUI
            thread th_tcpRecieve(&tcpRecieve);          // Start TCP Command Receiver Thread
            th_tcpRecieve.join();                       // Block until Remote Program Shutdown
            SOEM::inOP = false;                         // Start Program End Process
            cout << "\n\nRemote Program is off! ";
#else
            signal(SIGINT, interruptHandler);
            while (SOEM::inOP) sleep(1);                // Block until Interrupt Signal Detected
            cout << "\n\nInterrupt Signal Detected! ";
#endif

            cout << "Start End Process.\n\n";
            sleep(1);                          // Wait for Safe Return
            th_rtTask.rtLoopStop();            // Stop and Join Real-Time Control Thread
            th_ecatCheck.join();               // Join SOEM ecatCheck Thread 
        }

        SOEM::terminateEtherCAT();
    }
    cout << "End program\n";
    return 0;
}

