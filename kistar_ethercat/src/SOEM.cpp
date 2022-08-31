#include "SOEM.hpp"

#define EC_TIMEOUTMON 500

using namespace std;

char SOEM::IOmap[4096];
int SOEM::expectedWKC = 0;
int SOEM::wkc = 0;

// Flag //
bool SOEM::inOP = false;

// Functions //
/** Check that the correct number of devices are connected and init all slaves.
 *
 * @param[in] ifname              Dev name, f.e. "eth0"
*/
void SOEM::initializeEtherCAT(const char* ifname)
{
    // initialise SOEM, bind socket to ifname
    if (ec_init(ifname))
    {
        cout << "ec_init on " << ifname << " succeeded.\n";

        // find and auto-config slaves
        if (ec_config_init(FALSE) > 0)
        {
            cout << ec_slavecount << " slaves found and configured.\n\n";  // ec_slavecount -> slave num
            if (ec_slavecount != TOTAL_SLAVE_NUM)
            {
                cout << "WARNING : SLAVE NUMBER INSUFFICIENT\n\n";
                cout << "End program\n";
                ec_close();
                exit(0);
            }
        }
        else
        {
            cout << "No slaves found!\n\n";
            cout << "End program\n";
            ec_close();
            exit(0);
        }
    }
    else
    {
        cout << "No socket connection on " << ifname << "\nExecute as root\n\n";
        cout << "End program\n";
        exit(0);
    }

    for (int cnt = 1; cnt <= ec_slavecount; ++cnt)
    {
        cout << "Name: " << ec_slave[cnt].name << ", EEpMan: " << ec_slave[cnt].eep_man << ", eep_id: " << ec_slave[cnt].eep_id << "\n";
    }
}

/** Map PDOs of connected slave devices as defined in input parameter function.
 *
 * @param[in] *setup        pdo mapping function, f.e. "CPdoMapping::mapSlavePDOs"
*/
void SOEM::pdoMapping(int (*setup)(uint16 slaveIdx))
{
    for (int i = 1; i <= ec_slavecount; ++i)
    {
        ec_slave[i].PO2SOconfig = setup;
    }
    cout << "\nSlaves mapped!\n";
}

/** Change slave states to Operational. Turn inOP flag to true if all slaves become Operational.
*/
void SOEM::goingOperational()
{
    ec_config_map(&SOEM::IOmap);
    ec_configdc();

    // wait for all slaves to reach SAFE_OP state
    cout << "State to Safe-op...\n";
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    cout << "Calculated workcounter " << expectedWKC << endl;

    ec_dcsync0(1, TRUE, CONTROL_PERIOD_IN_ns, 0);  // SYNC0 on slave 1

    // sleep(2);

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    // send one valid process data to make outputs in slaves happy
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    // request OP state for all slaves
    ec_writestate(0);
    // wait for all slaves to reach OP state
    int chk = 200;
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        cout << "Operational state reached for all slaves!\n\n";
        inOP = true;
    }
    else { cout << "Not all slaves reached operational state.\n"; }
}

/** Map slave's input output address with user defined structs.
 *
 * @param[in] *servo            structure of mapped pdo
 * @param[in] num_of_servos     number of slave devices
*/
void SOEM::mapIOStructs(PDO_STRUCT servo[], int num_of_servos)
{
    for (int i = 0; i < num_of_servos; ++i)
    {
        servo[i].write = (PDO_WRITE*)(ec_slave[i + 1].outputs);
        servo[i].read = (PDO_READ*)(ec_slave[i + 1].inputs);
    }
}

/** Request INIT state for all slaves and stop SOEM, close ethercat socket.
*/
void SOEM::terminateEtherCAT()
{
    // request INIT state for all slaves
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);

    // stop SOEM, close socket
    ec_close();
}

/** SOEM providing function. Check the EtherCAT communication state, and recover when the error occured.
 * Run in thread separately.
*/
void *SOEM::ecatCheck() // SOEM providing function. Check the EtherCAT communication state, and recover when the error occured.
{
    while (inOP)
    {
        if ((wkc < expectedWKC) || ec_group[0].docheckstate)
        {
          /* one ore more slaves are not responding */
            ec_group[0].docheckstate = FALSE;
            ec_readstate();
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == 0) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[0].docheckstate = TRUE;
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
            if (!ec_group[0].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}
