/* Example code for Simple Open EtherCAT master *
 * Usage : motor_test [ifname1]                *
 * ifname is NIC interface, f.e. eth0           */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;


/********* PDO Objects Definition *********/
#define EPOS4 1

#pragma pack(push, 1)
typedef struct epos4_rx
{
    uint16_t  controlWord;
    int8_t    modeOfOperation;
    int16_t   targetTorque;
    int32_t   targetVelocity;
    int32_t   targetPosition;
    uint32_t  profileVelocity;
} epos4_rx;
typedef struct epos4_tx
{
    uint16_t  statusWord;
    int8_t    modeOfOperationDisplay;
    int32_t   positionActualValue;
    int32_t   velocityActualValue;
    int16_t   torqueActualValue;
    int32_t   velocityDemandValue;
} epos4_tx;
#pragma pack(pop)

epos4_rx* rxPDO[EPOS4];
epos4_tx* txPDO[EPOS4];


/** ---------- PDO Mapping ----------- **/
int EPOS4setup(uint16 slave)
{
    uint8 setZero = 0;
    uint8 rxPDO_size = 6;
    uint8 txPDO_size = 6;

    //     <0x1600> Receive PDO Mapping 1
    // 0x60400010 : Control Word               UInt16
    // 0x60600008 : Mode of Operation          Int8
    // 0x60710010 : Target Torque              Int16
    // 0x60ff0020 : Target Velocity            Int32
    // 0x607a0020 : Target Position            Int32
    // 0x60810020 : Profile Velocity           UInt32
    uint16 map_1c12[2] = { 0x0001, 0x1600 };
    uint32 map_1600[6] = { 0x60400010, 0x60600008, 0x60710010, 0x60ff0020, 0x607a0020, 0x60810020 };

    //     <0x1a00> Transmit PDO Mapping 1
    // 0x60410010 : Status Word                UInt16
    // 0x60610008 : Mode of Operation Display  Int8
    // 0x60640020 : Position Actual Value      Int32
    // 0x606c0020 : Velocity Actual Value      Int32
    // 0x60770010 : Torque Actual Value        Int16
    // 0x606b0020 : Velocity Demand Value      Int32
    uint16 map_1c13[2] = { 0x0001, 0x1a00 };
    uint32 map_1a00[6] = { 0x60410010, 0x60610008, 0x60640020, 0x606c0020, 0x60770010, 0x606b0020 };

    /* To map PDO in EPOS, the sequence is as follows */
    // 1. Write the value “0” (zero) to subindex 0x00 (disable PDO).
    ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(setZero), &setZero, EC_TIMEOUTSAFE * 4);
    ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(setZero), &setZero, EC_TIMEOUTSAFE * 4);
    ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(setZero), &setZero, EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, 0x1a00, 0x00, FALSE, sizeof(setZero), &setZero, EC_TIMEOUTSAFE);

    // 2. Modify the desired objects in subindex 0x01...0x0n.
    for (int i = 0; i < rxPDO_size; i++)
    {
        ec_SDOwrite(slave, 0x1600, 0x01 + i, FALSE, sizeof(map_1600[i]), &map_1600[i], EC_TIMEOUTSAFE);
    }
    for (int i = 0; i < txPDO_size; i++)
    {
        ec_SDOwrite(slave, 0x1a00, 0x01 + i, FALSE, sizeof(map_1a00[i]), &map_1a00[i], EC_TIMEOUTSAFE);
    }
    ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE * 4);
    ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE * 4);

    // 3. Write the No. of mapped object n in subindex 0x00
    ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(rxPDO_size), &rxPDO_size, EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, 0x1a00, 0x00, FALSE, sizeof(txPDO_size), &txPDO_size, EC_TIMEOUTSAFE);

    printf("EPOS4 slave %d set\n", slave);
    return 1;
}
/** ---------------------------------- **/

// void positionMode(epos4_rx* rxPDO, int32 iPosition)
// {
//     rxPDO->modeOfOperation = 0x08;
//     rxPDO->controlWord = 0x0f;
//     rxPDO->targetPosition = iPosition;
// }
void velocityMode(epos4_rx* rxPDO, epos4_tx* txPDO, int32 iVelocity)
{
    rxPDO->modeOfOperation = 0x09;
    rxPDO->controlWord = 0x0f;
    rxPDO->targetVelocity = iVelocity;

    printf("Velocity: %d, Torque: %d\r", txPDO->velocityActualValue, txPDO->torqueActualValue);
}
void torqueMode(epos4_rx* rxPDO, epos4_tx* txPDO, int16 iTorque)
{
    rxPDO->modeOfOperation = 0x0a;
    rxPDO->controlWord = 0x0f;
    rxPDO->targetTorque = iTorque;

    printf("Velocity: %d, Torque: %d\n", txPDO->velocityActualValue, txPDO->torqueActualValue);
}
// void homingMode(epos4_rx* rxPDO, int32 iPosition)
// {
//     rxPDO->modeOfOperation = 0x06;
//     rxPDO->controlWord = 0x0f;
//     rxPDO->targetPosition = iPosition;
// }




void simpletest(char *ifname)
{
    int i, oloop, iloop, chk;
    // int i, j, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    printf("Starting simple test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);

        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            /* link slave specific setup to preOP->safeOP hook */
            ec_slave[1].PO2SOconfig = EPOS4setup;
            printf("\nName: %s EEpMan: %d eep_id: %d configadr: %d aliasadr: %d State %d\n\r", ec_slave[1].name, ec_slave[1].eep_man, ec_slave[1].eep_id, ec_slave[1].configadr, ec_slave[1].aliasadr, ec_slave[1].state);

            ec_config_map(&IOmap);

            rxPDO[0] = (epos4_rx*)(ec_slave[1].outputs);
            txPDO[0] = (epos4_tx*)(ec_slave[1].inputs);

            printf("\nSlaves mapped, state to SAFE_OP.\n");

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 8)
                oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 8)
                iloop = 8;

            printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            printf("\nRequest operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* send one valid process data to make outputs in slaves happy */
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 200;

            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            /* Now we have a system up and running, all slaves are in state operational */
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("\nOperational state reached for all slaves.\n");
                inOP = TRUE;

                /* cyclic loop */
                for (i = 1; i <= 10000; i++)
                {
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if (wkc >= expectedWKC)
                    {
                        if ((txPDO[0]->statusWord & 0x08)) // Fault state
                        {
                            rxPDO[0]->controlWord = 0x80; // Fault reset
                        }
                        else if (i < 10)
                        {
                            rxPDO[0]->controlWord = 0x06; // Shut down
                        }
                        else if (!(txPDO[0]->statusWord & 0x02)) // Not switched on
                        {
                            rxPDO[0]->controlWord = 0x07; // Switch on
                        }
                        else if (!(txPDO[0]->statusWord & 0x04)) // Not operation enabled
                        {
                            rxPDO[0]->controlWord = 0x0f; // Operation enable
                        }
                        else
                        {
                            velocityMode(rxPDO[0], txPDO[0], 10000);
                            torqueMode(rxPDO[0], txPDO[0], 100);

                        }
                        printf("statusword %4x, controlword %x ", txPDO[0]->statusWord, rxPDO[0]->controlWord);

                            // printf("Processdata cycle %4d, WKC %d , O:", i, wkc);
                            // for (j = 0; j < oloop; j++)
                            // {
                            //     printf(" %2.2x", *(ec_slave[0].outputs + j));
                            // }
                            // printf(" I:");
                            // for (j = 0; j < iloop; j++)
                            // {
                            //     printf(" %2.2x", *(ec_slave[0].inputs + j));
                            // }
                            // printf(" T:%" PRId64 "\r", ec_DCtime);
                        needlf = TRUE;
                    }
                    osal_usleep(1000);
                }
                inOP = FALSE;
            }

            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
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

OSAL_THREAD_FUNC ecatcheck()
{
    int slave;

    while (1)
    {

        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
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
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1)
    {
        /* create thread to handle slave error handling in OP */
        // pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
        osal_thread_create(&thread1, 128000, (void*)&ecatcheck, NULL);
        /* start cyclic part */
        simpletest(argv[1]);
    }
    else
    {
        ec_adaptert *adapter = NULL;
        printf("Usage: motor_test ifname\nifname = eth0 for example\n");

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