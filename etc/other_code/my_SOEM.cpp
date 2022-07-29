#include "my_SOEM.hpp"

EPOS4::epos4_rx *rxPDO[EPOS_NUM];
EPOS4::epos4_tx *txPDO[EPOS_NUM];

// Variables //
char SOEM::IOmap[4096];
int SOEM::expectedWKC = 0;
volatile int SOEM::wkc = 0;
uint8 SOEM::currentgroup = 0;
int SOEM::chk = 0;

// Flags //
boolean SOEM::inOP = false;
boolean SOEM::needlf = false;
bool SOEM::ecatNumOk = false;
bool SOEM::ecatWKCOk = false;

// Functions //
/** Initialize lib in single NIC mode and init all slaves.
 *
 * @param[in] ifname              Dev name, f.e. "eth0"
 * @return                        true if succeed
*/
bool SOEM::initializeEtherCAT(const char* ifname)
{
    if (ec_init(ifname))
    {
        printf("EPOS4 : ec_init on %s succeeded.\n", ifname);
        // find and auto-config slaves
        // network discovery
        if (ec_config_init(FALSE) > 0) // TRUE when using configtable to init slaves, FALSE otherwise
        {
            printf("EPOS4 : %d slaves found and configured.\n", ec_slavecount); // ec_slavecount -> slave num
            if (ec_slavecount == EPOS_NUM)
            {
                ecatNumOk = true;
            }
            else
            {
                std::cout << "WARNING : SLAVE NUMBER INSUFFICIENT" << std::endl;
            }
            // CompleteAccess disabled for EPOS4 driver
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                //printf("EPOS4 : Has Slave[%d] CA? %s\n", slave, ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
                if (!(ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA))
                {
                    printf("EPOS4 : slave[%d] CA? : false , shutdown request \n ", slave);
                    return false;
                }
                ec_slave[slave].CoEdetails ^= ECT_COEDET_SDOCA;
            }
            return true;
        }
        else
        {
            printf("EPOS4 : Initializing slaves failed\n");
            return false;
        }
    }
    else
    {
        printf("Initialize lib with single NIC mode failed. Check dev name name ex)eth0, eth1 ...");
        return false;
    }
}

/** Map input output buffer and change slave states to Operational. Turn inOP flage to true if all slaves become Operational(NMT).
 * Exit program if failed.
*/
void SOEM::goingOperational()
{
    ec_config_map(&SOEM::IOmap);

    // usleep(1000*2000);
    // wait for all slaves to reach SAFE_OP state
    printf("EPOS4 : EC WAITING STATE TO SAFE_OP\n");
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

    printf("EPOS4 : Request operational state for all slaves. Calculated workcounter : %d\n", expectedWKC);
    if (expectedWKC == 3 * ec_slavecount)
    {
        ecatWKCOk = true;
    }
    else
    {
        std::cout << "WARNING : Calculated Workcounter insufficient!" << std::endl;
    }
    // usleep(1000*2000);
    // going operational
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    // send one valid process data to make outputs in slaves happy
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    // request OP state for all slaves
    ec_writestate(0);
    // wait for all slaves to reach OP state
    chk = 200;
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        printf("EPOS4 : All slaves reached operational state\n");
        // usleep(500000);
        for (int slave = 1; slave <= ec_slavecount; slave++)
        {
            rxPDO[slave - 1] = (EPOS4::epos4_rx *)(ec_slave[slave].outputs);
            txPDO[slave - 1] = (EPOS4::epos4_tx *)(ec_slave[slave].inputs);
        }
        inOP = true;
    }
    else
    {
        printf("EPOS4 : Failed to go operational state! Please restart\n");
        printf("EPOS4 : Exit Program\n");
        exit(0);
    }
}

/** SOEM providing function. Check the EtherCAT communication state, and recover when the error occured.
 * Run in thread separately.
*/
void *SOEM::ecatcheck(void *ptr) // SOEM providing function. Check the EtherCAT communication state, and recover when the error occured.
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
                      //  printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                      //  printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > 0)
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
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
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
                printf(".");
        }
        usleep(250);
    }
}

//////////////////////////////////////////////////////////////