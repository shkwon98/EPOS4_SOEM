#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <math.h>
#include <iostream>

#include "ethercat.h"
#include "schedDeadline.h"
#include "CPdoMapping.h"
#include "CTcpPacket.h"
#include "CUdpPacket.h"
#include "Macro.h"


using namespace std;

#define NUMOFWHEEL_DRIVE 1
#define CONTROL_PERIOD 1000.0 // us
#define DEADLINE 1000000 //deadline in ns

EPOS4_Drive_pt epos4_drive_pt[NUMOFWHEEL_DRIVE];

int started[NUMOFWHEEL_DRIVE] = { 0 };
uint servo_ready;

char IOmap[4096];
OSAL_THREAD_HANDLE thread1, thread2, thread3;
int expectedWKC;
bool needlf;
int wkc;
uint8 currentgroup = 0;
bool inOP = false;


bool bRunStart = false;


OSAL_THREAD_FUNC activationProcess(char *ifname)
{
    int chk, slc;
    needlf = false;

    printf("Starting activation process\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);

        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            if (ec_slavecount >= 1)
            {
                for (slc = 1; slc <= ec_slavecount; ++slc)
                {
                    printf("\nName: %s EEpMan: %d eep_id: %d State %d\n", ec_slave[slc].name, ec_slave[slc].eep_man, ec_slave[slc].eep_id, ec_slave[slc].state);

                    /* link slave specific setup to preOP->safeOP hook */
                    cPdoMapping.mapMotorPDOs_callback(slc);
                }
            }

            /* maps the previously mapped PDOs into the local buffer */
            ec_config_map(&IOmap);
            /* Configurate distributed clock */
            ec_configdc();

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
            printf("\nSlaves mapped, state to SAFE_OP...\n");

            /* connect struct pointers to slave I/O pointers */
            for (int i = 0; i < NUMOFWHEEL_DRIVE; ++i)
            {
                epos4_drive_pt[i].ptOutParam = (EPOS4_DRIVE_RxPDO_t*)(ec_slave[i + 1].outputs);
                epos4_drive_pt[i].ptInParam = (EPOS4_DRIVE_TxPDO_t*)(ec_slave[i + 1].inputs);
            }

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            // printf("Calculated workcounter %d\n", expectedWKC);

            /* send one valid process data to make outputs in slaves happy */
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            /* Call ec_dcsync0() to synchronize the slave and master clock */
            ec_dcsync0(1, TRUE, DEADLINE, 0); // SYNC0 on slave 1

            /* request OP state for all slaves */
            printf("Request operational state for all slaves...\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);

            /* wait for all slaves to reach OP state */
            ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);

            /* Now we have a system up and running, all slaves are in state operational */
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves!\n\n");
                inOP = true; // activate cyclic process

                while (1)
                {
                    osal_usleep(50000);
                }

                inOP = false;
            }

            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (int i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
                for (int i = 0; i < NUMOFWHEEL_DRIVE; i++)
                {
                    ec_dcsync0(i + 1, FALSE, 0, 0);
                }
            }

            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExecute as root\n", ifname);
    }
}



int main(int argc, char *argv[])
{
    cout << "SOEM (Simple Open EtherCAT Master)" << endl;
    cout << "< EPOS4 Motor Controller >\n" << endl;

    if (argc > 1)
    {
        /* create Non-RT thread (TCP Communication) */
        osal_thread_create(&thread1, 128000, (void*)&tcpCommunicate, NULL);

        /* create RT thread (Motor Control) */
        osal_thread_create(&thread2, 128000, (void*)&motorControl, NULL);

        /* create thread to handle slave error handling in OP */
        osal_thread_create(&thread3, 128000, (void*)&errorHandler, NULL);

        /* start acyclic part to activate the cyclic threads */
        activationProcess(argv[1]);
    }
    else
    {
        ec_adaptert *adapter = NULL;
        printf("Usage: main ifname\nifname = eth0 for example\n");

        printf("\nAvailable adapters:\n");
        adapter = ec_find_adapters();

        while (adapter != NULL)
        {
            printf("    - %s  (%s)\n", adapter->name, adapter->desc);
            adapter = adapter->next;
        }

        ec_free_adapters(adapter);
    }

    printf("End program\n");
    return (0);
}