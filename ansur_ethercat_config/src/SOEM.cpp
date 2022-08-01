#include "SOEM.hpp"

char SOEM::IOmap[4096];
int SOEM::chk = 0;
int SOEM::expectedWKC = 0;
int SOEM::wkc = 0;
uint8 SOEM::currentgroup = 0;

// Flag //
bool SOEM::inOP = false;

extern pthread_mutex_t mutex;


// Functions //
/** Initialize lib in single NIC mode and init all slaves.
 *
 * @param[in] ifname              Dev name, f.e. "eth0"
 * @return                        true if succeed
*/
void SOEM::initializeEtherCAT(const char* ifname)
{
    // initialise SOEM, bind socket to ifname
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);

        // find and auto-config slaves
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount); // ec_slavecount -> slave num
            if (ec_slavecount != EPOS4_NUM)
            {
                std::cout << "WARNING : SLAVE NUMBER INSUFFICIENT\n";
                std::cout << "End program\n";
                ec_close();
                exit(0);
            }
        }
        else
        {
            std::cout << "No slaves found!\n";
            std::cout << "End program\n";
            ec_close();
            exit(0);
        }
    }
    else
    {
        std::cout << "No socket connection on " << ifname << "\nExecute as root\n";
        std::cout << "End program\n";
        exit(0);
    }

    // for (int cnt = 1; cnt <= ec_slavecount; ++cnt)
    // {
    //     std::cout << "\nName: " << ec_slave[cnt].name << ", EEpMan: " << ec_slave[cnt].eep_man << ", eep_id: " << ec_slave[cnt].eep_id << std::endl;
    // }
}

void SOEM::goingSafeOP(int (*setup)(uint16 slaveIdx))
{
    for (int i = 1; i <= ec_slavecount; ++i)
    {
        ec_slave[i].PO2SOconfig = setup;
    }

    ec_config_map(&SOEM::IOmap);
    std::cout << "\nSlaves mapped!\n";
    ec_configdc();

    // wait for all slaves to reach SAFE_OP state
    std::cout << "State to Safe-op...\n";;
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
}


/** Change slave states to Operational. Turn inOP flage to true if all slaves become Operational(NMT).
 * Exit program if failed.
*/
void SOEM::goingOperational()
{
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    std::cout << "Calculated workcounter " << expectedWKC << std::endl;

    ec_dcsync0(1, TRUE, CONTROL_PERIOD_IN_ns, 0);  // SYNC0 on slave 1

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
        std::cout << "Operational state reached for all slaves!\n\n";
        inOP = true;
    }
    else { std::cout << "Not all slaves reached operational state.\n"; }
}


/** Map input output buffer Structs
*/
void SOEM::mapIOStructs(SERVO_IO::SERVO_WRITE *rxPDO[], SERVO_IO::SERVO_READ *txPDO[], int num_of_servos)
{
    for (int i = 1; i <= num_of_servos; ++i)
    {
        rxPDO[i - 1] = (SERVO_IO::SERVO_WRITE *)(ec_slave[i].outputs);
        txPDO[i - 1] = (SERVO_IO::SERVO_READ *)(ec_slave[i].inputs);
    }
}


void SOEM::terminateEtherCAT()
{
    // request INIT state for all slaves
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);

    // stop SOEM, close socket
    ec_close();
}


void *SOEM::ecatSync()
{
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    while (inOP)
    {
        pthread_mutex_lock(&mutex);
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        pthread_mutex_unlock(&mutex);

        next_time.tv_sec += (next_time.tv_nsec + CONTROL_PERIOD_IN_ns) / 1e9;
        next_time.tv_nsec = (int)(next_time.tv_nsec + CONTROL_PERIOD_IN_ns) % (int)1e9;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
}

/** SOEM providing function. Check the EtherCAT communication state, and recover when the error occured.
 * Run in thread separately.
*/
void *SOEM::ecatCheck() // SOEM providing function. Check the EtherCAT communication state, and recover when the error occured.
{
    while (inOP)
    {
        if ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)
        {
          /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (int slave = 1; slave <= ec_slavecount; slave++)
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
