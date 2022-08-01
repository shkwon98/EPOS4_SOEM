#include <iostream>
#include "CEcatMaster.h"
#include "Controller.h"
#include <functional>
#include <stdexcept>
#include <chrono>
#include <fstream>
#include <cmath>

using namespace std;
using namespace ecat;

int main(int argc, char *argv[]
{
    cout << "SOEM (Simple Open EtherCAT Master)\nStarting master...\n";

    if (argc > 1)
    {
        char *ifname = argv[1];
        uint16 state_check;





        try
        {
            CEcatMaster master(ifname, EC_TIMEOUT_TO_SAFE_OP);







            if (ec_slavecount >= 1)
            {
                for (slc = 1; slc <= ec_slavecount; ++slc)
                {
                    printf("\nName: %s EEpMan: %d eep_id: %d State %d\n", ec_slave[slc].name, ec_slave[slc].eep_man, ec_slave[slc].eep_id, ec_slave[slc].state);

                    /* link slave specific setup to preOP->safeOP hook */
                    cPdoMapping.mapMotorPDOs_callback(slc);
                }
            }













            master.configMap();
            master.configDC();









            try
            {
                master.movetoState(0, EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
                printf("\nSlaves mapped, state to SAFE_OP...\n");









                /* connect struct pointers to slave I/O pointers */
                for (int i = 0; i < NUMOFWHEEL_DRIVE; ++i)
                {
                    epos4_drive_pt[i].ptOutParam = (EPOS4_DRIVE_RxPDO_t*)getOutput_slave(i + 1);
                    epos4_drive_pt[i].ptInParam = (EPOS4_DRIVE_TxPDO_t*)getInput_slave(i + 1);
                }

                expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
                // printf("Calculated workcounter %d\n", expectedWKC);










                /* send one valid process data to make outputs in slaves happy */
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);














                master.config_ec_sync0(1, TRUE, DEADLINE, 0); // SYNC0 on slave 1

                try
                {










                    master.movetoState(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);







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










                    try
                    {
                        master.close_master();
                    }
                    catch (const runtime_error &e)
                    {
                        cout << "Error close_master\n";
                    }
                    master.waitThread();
                }
                catch (const runtime_error &e)
                {
                    cout << "Error state_transition SAFE_OP-> OP\n";
                }
            }
            catch (const runtime_error &e)
            {
                cout << "Error state_transition PRE_OP-> SAFE_OP\n";
            }
        }
        catch (const runtime_error &e)
        {
            cerr << e.what();
            return -1;
        }
    }
}
