#include "CEcatMaster.h"
// #include <stdexcept>
// #include <cstring>
// #include <fstream>

// #define NSEC_PER_SEC 1000000000

namespace ecat
{
    CEcatMaster::CEcatMaster(char *ifname)
    {
        initialize(ifname);
        config_init();
    }

    CEcatMaster::~CEcatMaster() {}

    //Initialise SOEM, bind socket to ifname
    void CEcatMaster::initialize(char *ifname)
    {
        if (ec_init(ifname))
        {
            std::cout << "ec_init on " << ifname << " succeeded.\n";
        }
        else
        {
            std::cout << "No socket connection on" << ifname << "\nExecute as root\n";
            exit(1);
        }
    }

    //Configuration and initialization of slave
    //return: number of slaves (0 if not slave)
    int CEcatMaster::config_init()
    {
        int slave_number = ec_config_init(FALSE);
        if (slave_number > 0)
        {
            std::cout << ec_slavecount << " slaves found and configured.\n";
            return slave_number;
        }
        else
        {
            std::cout << "No slaves found!\n";
            std::cout << "End simple test, close socket\n";
            CEcatMaster::close_master();
            exit(1);
        }
    }

    //Maps the previously mapped PDOs into the local buffer
    void CEcatMaster::configMap()
    {
        ec_config_map(&IOmap);
        std::cout << "\nSlaves mapped!\n";
    }

    //Configurate distributed clock
    bool CEcatMaster::configDC()
    {
        return ec_configdc();
    }

    //Change slave's state
    //Return: written state
    uint16 CEcatMaster::movetoState(uint16 slave, int state, int timeout)
    {
        ec_slave[slave].state = state;
        ec_writestate(slave);

        uint16 state_check = ec_statecheck(slave, state, timeout);

        switch (state)
        {
            case EC_STATE_INIT:
                std::cout << "State to Init...\n";;
                break;
            case EC_STATE_PRE_OP:
                std::cout << "State to Pre-op...\n";;
                break;
            case EC_STATE_BOOT:
                std::cout << "State to Boot...\n";;
                break;
            case EC_STATE_SAFE_OP:
                std::cout << "State to Safe-op...\n";;
                break;
            case EC_STATE_OPERATIONAL:
                std::cout << "State to Operational...\n";;
                break;
        }
        return state_check;
    }

    uint8 *CEcatMaster::getOutput_slave(uint16 position)
    {
        //std::cout<<"outputs_master: "<<ec_slave[position].outputs<<"\n";
        return ec_slave[position].outputs;
    }

    uint8 *CEcatMaster::getInput_slave(uint16 position)
    {
        //std::cout<<"inputs_master: "<<ec_slave[position].inputs<<"\n";
        return ec_slave[position].inputs;
    }

    // send one valid process data to make outputs in slaves happy
    void CEcatMaster::sendAndReceive(int timeout)
    {
        ec_send_processdata();
        ec_receive_processdata(timeout);
    }

    //Call ec_dcsync0() to synchronize the slave and master clock
    void CEcatMaster::config_ec_sync0(uint16 position, bool activate, uint32 cycletime, int cycleshift)
    {
        ec_dcsync0(position, activate, cycletime, cycleshift);
    }

    //Print slave's state
    void CEcatMaster::printState()
    {
        int cnt;
        //read and put the state in ec_slave[]
        ec_readstate();
        for (cnt = 1; cnt <= ec_slavecount; cnt++)
        {
            printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d, StatusCode=0x%4.4x : %s \n",
                   cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                   ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc,
                   ec_slave[cnt].ALstatuscode, ec_ALstatuscode2string(ec_slave[cnt].ALstatuscode));
        }
    }

    void CEcatMaster::close_master()
    {
        //movetoState_broadcast(EC_STATE_SAFE_OP, 200000);
        //movetoState_broadcast(EC_STATE_PRE_OP, 9000000);
        // shutdown = false;
        // thread = false;
        ec_close();
    }





















    // // PI calculation to get linux time synced to DC time
    // void CEcatMaster::ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
    // {
    //     static int64 integral = 0;
    //     int64 delta;

    //     delta = (reftime) % cycletime;
    //     if (delta > (cycletime / 2))
    //     {
    //         delta = delta - cycletime;
    //     }
    //     if (delta > 0)
    //     {
    //         integral++;
    //     }
    //     if (delta < 0)
    //     {
    //         integral--;
    //     }
    //     *offsettime = -(delta / 100) - (integral / 20);
    //     // g_delta = delta;
    // }

    // void CEcatMaster::createThread(int64 cycleTime)
    // {
    //     if (!thread)
    //     {
    //         //create real-time thread to exchange data
    //         setCycle(cycleTime);
    //         // shutdown = true;
    //         control_thread = std::thread(&CEcatMaster::ecatthread, this);
    //         thread = true;
    //     }
    //     else
    //         std::cout << "The thread already exists\n";
    // }

    // void CEcatMaster::setCycle(int64 cycletime)
    // {
    //     this->cycletime = cycletime;
    // }

    // void CEcatMaster::ecatthread()
    // {
    //     struct timespec ts, tleft;
    //     int ht;
    //     struct sched_attr attr;
    //     attr.size = sizeof(attr);
    //     sched_rr(&attr, 40, 0);

    //     clock_gettime(CLOCK_MONOTONIC, &ts);
    //     ht = (ts.tv_nsec / 1000000) + 1; // round to nearest ms
    //     ts.tv_nsec = ht * 1000000;
    //     toff = 0;

    //     if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    //     {
    //         std::cout << "mlockall failed\n";
    //         pthread_cancel(pthread_self());
    //     }

    //     //for (int i = 0; i < 5000; i++)
    //     i = 0;
    //     // while (shutdown)
    //     while (1)
    //     {
    //         time1 = ec_DCtime;
    //         this->add_timespec(&ts, cycletime + toff);
    //         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

    //         mtx.lock();
    //         ec_send_processdata();
    //         wkc = ec_receive_processdata(EC_TIMEOUTRET);
    //         mtx.unlock();

    //         if (ec_slave[0].hasdc)
    //         {
    //             // calculate toff to get linux time and DC synced
    //             this->ec_sync(ec_DCtime, cycletime, &toff);
    //         }

    //         time2 = ec_DCtime;
    //         cycle = time2 - time1;
    //         if (i < 50000)
    //             timecycle[i++] = cycle;
    //     }
    // }

    // void CEcatMaster::add_timespec(struct timespec *ts, int64 addtime)
    // {
    //     int64 sec, nsec;

    //     nsec = addtime % NSEC_PER_SEC;
    //     sec = (addtime - nsec) / NSEC_PER_SEC;
    //     ts->tv_sec += sec;
    //     ts->tv_nsec += nsec;
    //     if (ts->tv_nsec > NSEC_PER_SEC)
    //     {
    //         nsec = ts->tv_nsec % NSEC_PER_SEC;
    //         ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
    //         ts->tv_nsec = nsec;
    //     }
    // }














    // //Setup slave
    // void CEcatMaster::setupSlave(int slave, int (*setup)(uint16 position))
    // {
    //     //when the slave move from PRE_OP state to SAFE_OP state, it calls slave.setup to set parameters and map PDO
    //     ec_slave[slave].PO2SOconfig = setup;
    // }












    // void CEcatMaster::waitThread()
    // {
    //     control_thread.join();
    // }


//     void CEcatMaster::mutex_down()
//     {
//         mtx.lock();
//     }

//     void CEcatMaster::mutex_up()
//     {
//         mtx.unlock();
//     }

}