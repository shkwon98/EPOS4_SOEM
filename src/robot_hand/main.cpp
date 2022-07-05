#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <math.h>
#include <iostream>

#include "ethercat.h"
#include "ecatErrorHandle.h"
#include "scheduling.h"
#include "CPdoMapping.h"
#include "CTcpPacket.h"
#include "CUdpPacket.h"
#include "Macro.h"
#include "CEcatMaster.h"

CPdoMapping cPdoMapping;
CUdpPacket* pUdpPacket;
CTcpPacket* pTcpPacket;

using namespace ecat;
using namespace std;


#define EC_TIMEOUTMON 500
// #define EPOS4 1
#define NUMOF_DRIVE 1
#define CONTROL_PERIOD 1000.0 // us
#define DEADLINE 1000000 //deadline in ns

EPOS4_Drive_pt epos4_drive_pt[NUMOF_DRIVE];

int started[NUMOF_DRIVE] = { 0 };
uint servo_ready;

OSAL_THREAD_HANDLE thread1, thread2, thread3, thread4;

extern int expectedWKC;
extern volatile int wkc;
extern uint8 currentgroup;

bool inOP = false;

int64 toff = 0;

bool bRunStart = false;
short mode;
INPUT_LIST input;


/** Function that updates the deadline to synchronize mainThread **/
void refreshDeadline(struct sched_attr *attr, uint64 addTime)
{
    attr->sched_runtime = 0.95 * addTime;
    attr->sched_deadline = addTime;
    attr->sched_period = addTime;

    if (sched_setattr(0, attr, 0))
    {
        perror("sched_setattr failed");
        exit(1);
    }
}

/** PI calculation to get linux time synced to DC time **/
void ec_sync(int64 refTime, int64 cycleTime, int64 *offsetTime)
{
    static int64 integral = 0;
    int64 delta;

    delta = (refTime) % cycleTime;
    if (delta > (cycleTime / 2)) { delta = delta - cycleTime; }
    if (delta > 0) { integral++; }
    if (delta < 0) { integral--; }
    *offsetTime = -(delta / 100) - (integral / 20);
    // g_delta = delta;
}


bool ecat_activate(CEcatMaster master)
{
    if (ec_slavecount >= 1)
    {
        for (int slc = 1; slc <= ec_slavecount; ++slc)
        {
            cout << "\nName: " << ec_slave[slc].name << ", EEpMan: " << ec_slave[slc].eep_man << ", eep_id: " << ec_slave[slc].eep_id << endl;
            cPdoMapping.mapMotorPDOs_callback(slc);
        }
    }
    master.configDC();
    master.configMap();
    master.movetoState(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    /* connect struct pointers to slave I/O pointers */
    for (int i = 0; i < NUMOF_DRIVE; ++i)
    {
        epos4_drive_pt[i].ptOutParam = (EPOS4_DRIVE_RxPDO_t*)(master.getOutput_slave(i + 1));
        epos4_drive_pt[i].ptInParam = (EPOS4_DRIVE_TxPDO_t*)(master.getInput_slave(i + 1));
    }
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    // cout << "Calculated workcounter " << expectedWKC << endl;

    master.sendAndReceive(EC_TIMEOUTRET);
    master.config_ec_sync0(1, TRUE, DEADLINE, 0); // SYNC0 on slave 1

    cout << "Request operational state for all slaves...\n";
    master.movetoState(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);  // request OP state for all slaves
    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        cout << "Operational state reached for all slaves!\n\n";
        inOP = true; // activate cyclic process

        while (1)
        {
            sleep(1);
        }
    }
    else
    {
        cout << "Not all slaves reached operational state.\n";

        master.printState();
        for (int i = 0; i < NUMOF_DRIVE; i++)
        {
            master.config_ec_sync0(i + 1, FALSE, 0, 0);
        }
        inOP = false;
    }
    return inOP;
}


double sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now)
{
    double a = pos_init;
    double b = (pos_fin - pos_init);
    double c = (M_PI / ((time_fin - time_init) * 2.0));
    double d = (-1.0) * (time_init);

    return (a + b * sin(c * (time_now + d)));
}

OSAL_THREAD_FUNC tcpCommunicate()
{
    pTcpPacket = new CTcpPacket();

    while (1)  // 10ms non-rt loop
    {
        if (inOP && pTcpPacket->readPacket() > 0)
        {
            mode = pTcpPacket->getHeader();
            switch (mode)
            {
                case COMMAND_SET_TASK_PARAM:
                    pTcpPacket->decode(input.taskParam);
                    break;

                case COMMAND_RUN_CSP:
                    pTcpPacket->decode(bRunStart);
                    break;

                case COMMAND_RUN_CSV:
                    pTcpPacket->decode(bRunStart);
                    pTcpPacket->decode(input.velocity);
                    break;

                case COMMAND_RUN_CST:
                    pTcpPacket->decode(bRunStart);
                    pTcpPacket->decode(input.torque);
                    break;
            }
        }
        osal_usleep(10000);
    }

    delete pTcpPacket;
}

OSAL_THREAD_FUNC motorControl()
{
    double t_loopStart, t_lastLoopStart, t_taskEnd;
    struct timespec ts;

    struct sched_attr attr;
    attr.size = sizeof(attr);
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_flags = 0;
    attr.sched_nice = 0;
    attr.sched_priority = 0;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        cout << "mlockall failed: %m" << endl;
        pthread_cancel(pthread_self());
    }

    pUdpPacket = new CUdpPacket;

    unsigned int tick = 0;
    double curPos = 0.0;
    double tarPos = 0.0;
    bool bEndFlag = true;
    double endCnt = 0;

    int i;
    uint16 controlword = 0;

    while (1)
    {
        /* calculate next cycle start */
        refreshDeadline(&attr, (uint64)(DEADLINE + toff));
        /* wait to cycle start */
        sched_yield();

        if (inOP)
        {
            ////////////////////////////////////////////////////////////////////
            //////////////////////  LOOP TIME MEASUREMENT  /////////////////////
            ////////////////////////////////////////////////////////////////////
            //
            clock_gettime(CLOCK_MONOTONIC, &ts);
            t_loopStart = ts.tv_nsec;
            // printf("\r| [Loop time: %.4lfms], ", (t_loopStart - t_lastLoopStart) / 1000000.0);
            t_lastLoopStart = t_loopStart;

            ////////////////////////////////////////////////////////////////////
            ///////////////////////////  LOOP TASKS  ///////////////////////////
            ////////////////////////////////////////////////////////////////////
            //
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            servo_ready = 0;
            for (i = 0; i < NUMOF_DRIVE; ++i)
            {
                controlword = 0;
                started[i] = ServoOn_GetCtrlWrd(epos4_drive_pt[i].ptInParam->StatusWord, &controlword);

                epos4_drive_pt[i].ptOutParam->ControlWord = controlword;

                servo_ready += started[i];
            }

            /* BEGIN USER CODE */
            if (servo_ready == NUMOF_DRIVE) // The given task begins here
            {
                switch (input.taskParam.taskType)
                {
                    case RUN_TASK_NONE:
                        tarPos = 0;
                        break;

                    case RUN_TASK_MOTOR_CONTROL:
                    {
                        double dTriPeriod = (2 * M_PI) / (input.taskParam.period * CONTROL_PERIOD);
                        tarPos = input.taskParam.disp * sin(dTriPeriod * tick++);
                        break;
                    }

                    default:
                        tarPos = 0;
                        break;
                }

                if (bRunStart)  // Control ON
                {
                    switch (mode)
                    {
                        case COMMAND_RUN_CSP:  // Position Control
                            epos4_drive_pt[0].ptOutParam->ModeOfOperation = OP_MODE_CYCLIC_SYNC_POSITION;
                            epos4_drive_pt[0].ptOutParam->TargetPosition = (int)CONV_MM_to_INC(tarPos);
                            break;

                        case COMMAND_RUN_CSV:  // Velocity Control
                            epos4_drive_pt[0].ptOutParam->ModeOfOperation = OP_MODE_CYCLIC_SYNC_VELOCITY;
                            epos4_drive_pt[0].ptOutParam->TargetVelocity = input.velocity;
                            break;

                        case COMMAND_RUN_CST:  // Torque Control
                            epos4_drive_pt[0].ptOutParam->ModeOfOperation = OP_MODE_CYCLIC_SYNC_TORQUE;
                            epos4_drive_pt[0].ptOutParam->TargetTorque = input.torque;
                            break;
                    }
                    bEndFlag = true;
                }

                else  // Control OFF
                {
                    tick = 0;

                    // Initialization
                    if (bEndFlag)
                    {
                        curPos = CONV_INC_to_MM(epos4_drive_pt[0].ptInParam->PositionActualValue);
                        bEndFlag = false;
                        endCnt = 0;
                    }

                    tarPos = sin_motion(curPos, 0.0, 0.0, 2000.0, endCnt++);
                    if (endCnt >= 2000)
                    {
                        tarPos = 0;
                    }

                    epos4_drive_pt[0].ptOutParam->ModeOfOperation = OP_MODE_CYCLIC_SYNC_POSITION;
                    epos4_drive_pt[0].ptOutParam->TargetPosition = (int)CONV_MM_to_INC(tarPos);
                }
            }
            // cout << "[Target Position: " << epos4_drive_pt[0].ptOutParam->TargetPosition << "] ";
            // cout << "| [Actual Position : " << epos4_drive_pt[0].ptInParam->PositionActualValue << "] " << endl;

            short header = STREAM_MODE;
            LOG_DATA logData;

            logData.actualVelocity = epos4_drive_pt[0].ptInParam->VelocityActualValue;
            logData.actualTorque = epos4_drive_pt[0].ptInParam->TorqueActualValue;
            logData.actualPosition = epos4_drive_pt[0].ptInParam->PositionActualValue;

            pUdpPacket->setCommandHeader(header);
            pUdpPacket->encode(logData);
            pUdpPacket->sendPacket();

            if (ec_slave[0].hasdc)
            {
                /* calculate toff to get linux time and DC synced */
                ec_sync(ec_DCtime, DEADLINE, &toff);
            }

            /********************** TASK TIME MEASUREMENT **********************/
            clock_gettime(CLOCK_MONOTONIC, &ts);
            t_taskEnd = ts.tv_nsec;
            // printf("[Task time: %.4lfms] |   ", (t_taskEnd - t_loopStart) / 1000000.0);
            // fflush(stdout);
        }
    }

    delete pUdpPacket;

    printf("End motor test, close socket\n");
    ec_close();
}



int main(int argc, char *argv[])
{
    cout << "SOEM (Simple Open EtherCAT Master)\n< EPOS4 Motor Controller >" << endl;

    if (argc > 1)
    {
        /* create thread to handle slave error handling in OP */
        osal_thread_create(&thread1, 128000, (void*)&errorHandle, NULL);

        /* create Non-RT thread (TCP Communication) */
        osal_thread_create(&thread2, 128000, (void*)&tcpCommunicate, NULL);

        /* create RT thread (Motor Control) */
        osal_thread_create(&thread3, 128000, (void*)&motorControl, NULL);

        CEcatMaster master(argv[1]);

        ecat_activate(master);


        master.movetoState(0, EC_STATE_INIT, EC_TIMEOUTSTATE);
        master.close_master();
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