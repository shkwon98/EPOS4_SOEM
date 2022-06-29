#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <math.h>

#include "ethercat.h"
#include "schedDeadline.h"
#include "tcpPacket.h"
#include "udpPacket.h"
#include "Macro.h"


#define PI  3.14159265359
#define DEADLINE 900000 //deadline in ns
#define EC_TIMEOUTMON 500
#define EPOS4 1


char IOmap[4096];
OSAL_THREAD_HANDLE thread1, thread2, thread3;
int expectedWKC;
bool needlf;
volatile int wkc;
bool inOP;
uint8 currentgroup = 0;

bool dorun = false;
int64 toff, g_delta;

RX_PDO* rxPDO;
TX_PDO* txPDO;

UDP_Packet* pUdpPacket;
TCP_Packet* pTcpPacket;

short mode;
INPUT_LIST input;


/** Function for PDO mapping **/
void PdoMapping(uint16 slave)
{
    uint8 PDO_abled = 0;
    uint8 RxPDOs_number = 0;
    uint8 TxPDOs_number = 0;

    /* <0x1600> Receive PDO Mapping 1 */
    OBJ RxPDO1 = { 0x1600, 0x01, sizeof(uint32), 0x60400010 };   // 0x60400010 : control_word               UInt16
    OBJ RxPDO2 = { 0x1600, 0x02, sizeof(uint32), 0x60600008 };   // 0x60600008 : mode_of_operation          Int8
    OBJ RxPDO3 = { 0x1600, 0x03, sizeof(uint32), 0x60710010 };   // 0x60710010 : target_torque              Int16
    OBJ RxPDO4 = { 0x1600, 0x04, sizeof(uint32), 0x60ff0020 };   // 0x60ff0020 : target_velocity            Int32
    OBJ RxPDO5 = { 0x1600, 0x05, sizeof(uint32), 0x607a0020 };   // 0x607a0020 : target_position            Int32
    OBJ RxPDO6 = { 0x1600, 0x06, sizeof(uint32), 0x60810020 };   // 0x60810020 : profile_velocity           Int32
    // OBJ RxPDO7 = { 0x1600, 0x07, sizeof(), 0x };
    // OBJ RxPDO8 = { 0x1600, 0x08, sizeof(), 0x };
    OBJ SM2_choose_RxPDO = { 0x1C12, 0x01, sizeof(uint16), RxPDO1.index };

    /* <0x1A00> Transmit PDO Mapping 1 */
    OBJ TxPDO1 = { 0x1A00, 0x01, sizeof(uint32), 0x60410010 };   // 0x60410010 : status_word                UInt16
    OBJ TxPDO2 = { 0x1A00, 0x02, sizeof(uint32), 0x60610008 };   // 0x60610008 : mode_of_operation_display  Int8
    OBJ TxPDO3 = { 0x1A00, 0x03, sizeof(uint32), 0x60640020 };   // 0x60640020 : position_actual_value      Int32
    OBJ TxPDO4 = { 0x1A00, 0x04, sizeof(uint32), 0x606C0020 };   // 0x606C0020 : velocity_actual_value      Int32
    OBJ TxPDO5 = { 0x1A00, 0x05, sizeof(uint32), 0x60770010 };   // 0x60770010 : torque_actual_value        Int16
    OBJ TxPDO6 = { 0x1A00, 0x06, sizeof(uint32), 0x30D10220 };   // 0x30D10220 : current_actual_value       Int32
    // OBJ TxPDO7 = { 0x1A00, 0x07, sizeof(), 0x };
    // OBJ TxPDO8 = { 0x1A00, 0x08, sizeof(), 0x };
    OBJ SM3_choose_TxPDO = { 0x1C13, 0x01, sizeof(uint16), TxPDO1.index };

    // 1. Write the value “0” (zero) to subindex 0x00 (disable PDO).
    ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(PDO_abled), &PDO_abled, EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(PDO_abled), &PDO_abled, EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, 0x1C12, 0x00, FALSE, sizeof(PDO_abled), &PDO_abled, EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(PDO_abled), &PDO_abled, EC_TIMEOUTSAFE);

    // 2. Modify the desired objects in subindex 0x01...0x0n.
    ec_SDOwrite(slave, RxPDO1.index, RxPDO1.sub_index, FALSE, RxPDO1.size, &(RxPDO1.value), EC_TIMEOUTSAFE);
    RxPDOs_number++;
    ec_SDOwrite(slave, RxPDO2.index, RxPDO2.sub_index, FALSE, RxPDO2.size, &(RxPDO2.value), EC_TIMEOUTSAFE);
    RxPDOs_number++;
    ec_SDOwrite(slave, RxPDO3.index, RxPDO3.sub_index, FALSE, RxPDO3.size, &(RxPDO3.value), EC_TIMEOUTSAFE);
    RxPDOs_number++;
    ec_SDOwrite(slave, RxPDO4.index, RxPDO4.sub_index, FALSE, RxPDO4.size, &(RxPDO4.value), EC_TIMEOUTSAFE);
    RxPDOs_number++;
    ec_SDOwrite(slave, RxPDO5.index, RxPDO5.sub_index, FALSE, RxPDO5.size, &(RxPDO5.value), EC_TIMEOUTSAFE);
    RxPDOs_number++;
    ec_SDOwrite(slave, RxPDO6.index, RxPDO6.sub_index, FALSE, RxPDO6.size, &(RxPDO6.value), EC_TIMEOUTSAFE);
    RxPDOs_number++;
    // ec_SDOwrite(slave, RxPDO7.index, RxPDO7.sub_index, FALSE, RxPDO7.size, &(RxPDO7.value), EC_TIMEOUTSAFE);
    // RxPDOs_number++;
    // ec_SDOwrite(slave, RxPDO8.index, RxPDO8.sub_index, FALSE, RxPDO8.size, &(RxPDO8.value), EC_TIMEOUTSAFE);
    // RxPDOs_number++;

    ec_SDOwrite(slave, TxPDO1.index, TxPDO1.sub_index, FALSE, TxPDO1.size, &(TxPDO1.value), EC_TIMEOUTSAFE);
    TxPDOs_number++;
    ec_SDOwrite(slave, TxPDO2.index, TxPDO2.sub_index, FALSE, TxPDO2.size, &(TxPDO2.value), EC_TIMEOUTSAFE);
    TxPDOs_number++;
    ec_SDOwrite(slave, TxPDO3.index, TxPDO3.sub_index, FALSE, TxPDO3.size, &(TxPDO3.value), EC_TIMEOUTSAFE);
    TxPDOs_number++;
    ec_SDOwrite(slave, TxPDO4.index, TxPDO4.sub_index, FALSE, TxPDO4.size, &(TxPDO4.value), EC_TIMEOUTSAFE);
    TxPDOs_number++;
    ec_SDOwrite(slave, TxPDO5.index, TxPDO5.sub_index, FALSE, TxPDO5.size, &(TxPDO5.value), EC_TIMEOUTSAFE);
    TxPDOs_number++;
    ec_SDOwrite(slave, TxPDO6.index, TxPDO6.sub_index, FALSE, TxPDO6.size, &(TxPDO6.value), EC_TIMEOUTSAFE);
    TxPDOs_number++;
    // ec_SDOwrite(slave, TxPDO7.index, TxPDO7.sub_index, FALSE, TxPDO7.size, &(TxPDO7.value), EC_TIMEOUTSAFE);
    // RxPDOs_number++;
    // ec_SDOwrite(slave, TxPDO8.index, TxPDO8.sub_index, FALSE, TxPDO8.size, &(TxPDO8.value), EC_TIMEOUTSAFE);
    // RxPDOs_number++;

    // 3. Write the No. of mapped object n in subindex 0x00
    ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(RxPDOs_number), &(RxPDOs_number), EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(TxPDOs_number), &(TxPDOs_number), EC_TIMEOUTSAFE);

    // 4. Make the changes effective
    PDO_abled = 1;
    ec_SDOwrite(slave, SM2_choose_RxPDO.index, SM2_choose_RxPDO.sub_index, FALSE, SM2_choose_RxPDO.size, &(SM2_choose_RxPDO.value), EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, SM3_choose_TxPDO.index, SM3_choose_TxPDO.sub_index, FALSE, SM3_choose_TxPDO.size, &(SM3_choose_TxPDO.value), EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, 0x1C12, 0x00, FALSE, sizeof(PDO_abled), &(PDO_abled), EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(PDO_abled), &(PDO_abled), EC_TIMEOUTSAFE);
}

/** Function for parameter setup and PdoMapping call **/
int SetupParam(uint16 slave)
{
    // OBJ velocity_unit = { 0x60A9, 0x00, sizeof(uint32), (uint32)0x00B44700 };

    // /* SETUP of default values */
    // ec_SDOwrite(slave, velocity_unit.index, velocity_unit.sub_index, FALSE, velocity_unit.size, &(velocity_unit.value), EC_TIMEOUTSAFE);

    //  /* Check that the parameters are set correctly */
    // int retval;
    // uint32 velocity_unit_value;
    // retval = ec_SDOread(slave, velocity_unit.index, velocity_unit.sub_index, FALSE, &(velocity_unit.size), &velocity_unit_value, EC_TIMEOUTSAFE);
    // printf("velocity unit=%u,size=%d,return=%d\n", velocity_unit_value, velocity_unit.size, retval);

    /* PDO mapping */
    PdoMapping(EPOS4);

    printf("\nEPOS4 slave %d set\n", slave);
    return 1;
}

/** Function that updates the deadline to synchronize ecatthread **/
void RefreshDeadline(struct sched_attr *attr, uint64 addtime)
{
    attr->sched_runtime = 0.95 * addtime;
    attr->sched_deadline = addtime;
    attr->sched_period = addtime;

    if (sched_setattr(gettid(), attr, 0))
    {
        perror("sched_setattr failed");
        exit(1);
    }
}

/** PI calculation to get linux time synced to DC time **/
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;

    delta = (reftime) % cycletime;
    if (delta > (cycletime / 2)) { delta = delta - cycletime; }
    if (delta > 0) { integral++; }
    if (delta < 0) { integral--; }
    *offsettime = -(delta / 100) - (integral / 20);
    g_delta = delta;
}


OSAL_THREAD_FUNC simpletest(char *ifname)
{
    needlf = false;
    inOP = false;

    printf("Starting simple test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);

        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);
            printf("\nName: %s EEpMan: %d eep_id: %d State %d\n\r", ec_slave[EPOS4].name, ec_slave[EPOS4].eep_man, ec_slave[EPOS4].eep_id, ec_slave[EPOS4].state);

            /* link slave specific setup to preOP->safeOP hook */
            ec_slave[EPOS4].PO2SOconfig = SetupParam;

            /* Locate DC slaves, measure propagation delays. */
            ec_configdc();

            // maps the previously mapped PDOs into the local buffer
            ec_config_map(&IOmap);

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
            printf("\nSlaves mapped, state to SAFE_OP.\n");

            /* connect struct pointers to slave I/O pointers */
            rxPDO = (RX_PDO*)(ec_slave[EPOS4].outputs);
            txPDO = (TX_PDO*)(ec_slave[EPOS4].inputs);

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            printf("\nRequest operational state for all slaves\n");

            /* send one processdata cycle to init SM in slaves */
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            /* Call ec_dcsync0() to synchronize the slave and master clock */
            ec_dcsync0(EPOS4, TRUE, DEADLINE, 0); // SYNC0 on slave 1

            /* request OP state for all slaves */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);

            /* wait for all slaves to reach OP state */
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 5);

            /* Now we have a system up and running, all slaves are in state operational */
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("\nOperational state reached for all slaves.\n\n");
                inOP = true;
                dorun = true; // activate cyclic rt process

                pTcpPacket = new TCP_Packet();

                /* acyclic loop 10ms */
                while (1)
                {
                    if (pTcpPacket->readPacket() > 0)
                    {
                        mode = pTcpPacket->getHeader();
                        switch (mode)
                        {
                            case COMMAND_MODE_CSV:
                                pTcpPacket->decode(input.velocity);
                                break;
                            case COMMAND_MODE_CST:
                                pTcpPacket->decode(input.torque);
                                break;
                            case COMMAND_MODE_PPM:
                                pTcpPacket->decode(input.position);
                                break;
                            case COMMAND_MODE_CSP:
                                pTcpPacket->decode(input.position);
                                break;
                            case COMMAND_MODE_SINUSOIDAL_VELOCITY:
                                pTcpPacket->decode(input.sine_value);
                                break;
                            case COMMAND_MODE_SET_ZERO:
                                break;
                        }
                    }
                    osal_usleep(10000);
                }

                delete pTcpPacket;
                dorun = false;
                inOP = false;
                sleep(1);
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
            }
            printf("Request safe operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_SAFE_OP;
            /* request SAFE_OP state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExecute as root\n", ifname);
    }
}


/** RT EtherCAT thread **/
OSAL_THREAD_FUNC ecatthread()
{
    double t_loopStart, t_lastLoopStart, t_taskEnd;
    struct timespec ts;

    // printf("\nRT ecatthread started [%ld]\n", gettid());

    struct sched_attr attr;
    attr.size = sizeof(attr);
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_flags = 0;
    attr.sched_nice = 0;
    attr.sched_priority = 0;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        printf("mlockall failed: %m\n");
        pthread_cancel(pthread_self());
    }

    pUdpPacket = new UDP_Packet;

    toff = 0;
    dorun = 0;
    ec_send_processdata();
    int cnt = 0;
    int cnt2 = 0;
    int cwFlag = 1;
    while (1)
    {
        /* calculate next cycle start */
        RefreshDeadline(&attr, (uint64)(DEADLINE + toff));
        /* wait to cycle start */
        sched_yield();

        if (dorun)
        {
            /********************** LOOP TIME MEASUREMENT **********************/
            clock_gettime(CLOCK_MONOTONIC, &ts);
            t_loopStart = ts.tv_nsec;
            printf("\r| [Loop time: %.4lfms], ", (t_loopStart - t_lastLoopStart) / 1000000.0);
            t_lastLoopStart = t_loopStart;

            /*************************** LOOP TASKS ***************************/
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            /* BEGIN USER CODE */
            if ((txPDO->status_word & FAULT))
            {
                rxPDO->control_word = FAULT_RESET;
            }
            else if ((txPDO->status_word & SWITCH_ON_DISABLED)) // Switch on disabled state
            {
                rxPDO->control_word = SHUTDOWN; // Shut down
            }
            else if (!(txPDO->status_word & SWITCHED_ON)) // Not switched on
            {
                rxPDO->control_word = SWITCH_ON; // Switch on
            }
            else if (!(txPDO->status_word & OPERATION_ENABLED)) // Not operation enabled
            {
                rxPDO->control_word = SWITCH_ON_ENABLE; // Operation enable
            }
            else
            {
                switch (mode)
                {
                    case COMMAND_MODE_STOP_MOTOR:
                        rxPDO->mode_of_operation = CSV;
                        rxPDO->target_velocity = 0;
                        break;
                    case COMMAND_MODE_CSV:
                        rxPDO->mode_of_operation = CSV;
                        rxPDO->target_velocity = input.velocity;
                        break;
                    case COMMAND_MODE_CST:
                        rxPDO->mode_of_operation = CST;
                        rxPDO->target_torque = input.torque;
                        break;
                    case COMMAND_MODE_PPM:
                        if ((cnt % 4) == 0)
                        {
                            rxPDO->control_word = 0x3F;
                            rxPDO->mode_of_operation = PPM;
                            rxPDO->target_position = input.position;

                            if (txPDO->status_word & 0x1000)
                            {
                                rxPDO->control_word = 0x0F;
                            }
                        }
                        cnt++;
                        break;
                    case COMMAND_MODE_CSP:
                        rxPDO->mode_of_operation = CSP;
                        rxPDO->target_position = input.position;

                        break;

                    case COMMAND_MODE_SINUSOIDAL_VELOCITY:
                        if (cnt <= 20000)
                        {
                            rxPDO->mode_of_operation = CSV;
                            rxPDO->target_velocity = input.sine_value.amplitude * sin((2 * PI * input.sine_value.frequency) / 1000 * cnt++);
                        }
                        else
                            cnt = 0;
                        break;
                    case COMMAND_MODE_BACK_AND_FORTH_VELOCITY:
                        rxPDO->mode_of_operation = CSV;
                        rxPDO->target_velocity = cnt2;
                        if (cwFlag)
                        {
                            if (cnt2 < 20000)
                                cnt2 += 10;
                            else
                                cwFlag = 0;
                        }
                        else
                        {
                            if (cnt2 > -20000)
                                cnt2 -= 10;
                            else
                                cwFlag = 1;
                        }
                        break;
                    case COMMAND_MODE_SET_ZERO:
                        if (txPDO->position_actual_value < -200000)
                        {
                            rxPDO->mode_of_operation = CSV;
                            rxPDO->target_velocity = 20000;
                        }
                        else if (txPDO->position_actual_value > 200000)
                        {
                            rxPDO->mode_of_operation = CSV;
                            rxPDO->target_velocity = -20000;
                        }
                        else
                        {
                            rxPDO->target_velocity = -(txPDO->position_actual_value / 10);
                            if (txPDO->velocity_actual_value == 0)
                            {
                                input.position = 0;
                                mode = COMMAND_MODE_PPM;
                            }
                        }
                        break;
                }
            }
            // printf("| [Velocity: %6d], [Torque: %5d] ", txPDO->velocity_actual_value, txPDO->torque_actual_value);

            short header = STREAM_MODE;
            LOG_DATA logData;
            logData.velocity_actual_value = txPDO->velocity_actual_value;
            logData.torque_actual_value = txPDO->torque_actual_value;
            logData.position_actual_value = txPDO->position_actual_value;
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
            printf("[Task time: %.4lfms] |   ", (t_taskEnd - t_loopStart) / 1000000.0);
            fflush(stdout);
        }
    }

    delete pUdpPacket;

    printf("End motor test, close socket\n");
    ec_close();
}


OSAL_THREAD_FUNC ecatcheck()
{
    int slave;

    while (1)
    {

        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = false;
                printf("\n");
            }

            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();

            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;

                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);

                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }

                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }

            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }

        osal_usleep(10000);
    }
}


int main(int argc, char *argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\n< EPOS4 motor control >\n");

    if (argc > 1)
    {
        /* create RT thread */
        osal_thread_create(&thread1, 128000, (void*)&ecatthread, NULL);

        /* create thread to handle slave error handling in OP */
        osal_thread_create(&thread2, 128000, (void*)&ecatcheck, NULL);

        /* start acyclic part */
        simpletest(argv[1]);
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