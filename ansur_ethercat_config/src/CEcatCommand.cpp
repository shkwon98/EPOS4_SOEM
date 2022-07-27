#include "CEcatCommand.hpp"

int servo_enable(uint16 StatusWord, uint16 * ControlWord)
{
    int  _enable = 0;
    if (bit_is_clear(StatusWord, STATUSWORD_OPERATION_ENABLE_BIT)) //Not ENABLED yet
    {
        if (bit_is_clear(StatusWord, STATUSWORD_SWITCHED_ON_BIT)) //Not SWITCHED ON yet
        {
            if (bit_is_clear(StatusWord, STATUSWORD_READY_TO_SWITCH_ON_BIT)) //Not READY to SWITCH ON yet
            {
                if (bit_is_set(StatusWord, STATUSWORD_FAULT_BIT)) //FAULT exist
                {
                    (*ControlWord) = 0x80;	//FAULT RESET command
                }
                else //NO FAULT
                {
                    (*ControlWord) = 0x06;	//SHUTDOWN command (transition#2)
                }
            }
            else //READY to SWITCH ON
            {
                (*ControlWord) = 0x07;	//SWITCH ON command (transition#3)
            }
        }
        else //has been SWITCHED ON
        {
            (*ControlWord) = 0x0F;	//ENABLE OPERATION command (transition#4)
            _enable = 1;
        }
    }
    else //has been ENABLED
    {
        (*ControlWord) = 0x0F;	//maintain OPERATION state
        _enable = 1;
    }
    return _enable;;
}


namespace ecat
{
    CEcatMaster::CEcatMaster(char *ifname)
    {
        initialize(ifname);
        config_init();
    }

    CEcatMaster::~CEcatMaster() {}

    // Initialise SOEM, bind socket to ifname
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

    // Configuration and initialization of slave
    // return: number of slaves (0 if not slave)
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

    // Setup slave
    void CEcatMaster::setupPDO(int slave, int (*setup)(uint16 slaveIdx))
    {
        ec_slave[slave].PO2SOconfig = setup;
    }

    // Maps the previously mapped PDOs into the local buffer
    void CEcatMaster::configMap()
    {
        ec_config_map(&IOmap);
        std::cout << "\nSlaves mapped!\n";
    }

    // Configurate distributed clock
    bool CEcatMaster::configDC()
    {
        return ec_configdc();
    }

    // Change slave's state
    // Return: written state
    uint16 CEcatMaster::movetoState(uint16 slave, uint16 state, int timeout)
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

    /* connect struct pointers to slave I/O pointers */
    void CEcatMaster::map_structs(SERVO_IO_pt servo[], int num_of_servos)
    {
        for (int i = 0; i < num_of_servos; ++i)
        {
            servo[i].writeParam = (SERVO_WRITE*)(ec_slave[i + 1].outputs);
            servo[i].readParam = (SERVO_READ*)(ec_slave[i + 1].inputs);
        }
    }

    // send one valid process data to make outputs in slaves happy
    void CEcatMaster::sendAndReceive(int timeout)
    {
        ec_send_processdata();
        ec_receive_processdata(timeout);
    }

    // Call ec_dcsync0() to synchronize the slave and master clock
    void CEcatMaster::config_ec_sync0(uint16 slave, bool activate, uint32 cycletime, int cycleshift)
    {
        ec_dcsync0(slave, activate, cycletime, cycleshift);
    }

    // Print slave's state
    void CEcatMaster::printState1()
    {
        for (int cnt = 1; cnt <= ec_slavecount; ++cnt)
        {
            std::cout << "\nName: " << ec_slave[cnt].name << ", EEpMan: " << ec_slave[cnt].eep_man << ", eep_id: " << ec_slave[cnt].eep_id << std::endl;
        }
    }

    // Print slave's state
    void CEcatMaster::printState2()
    {
        // read and put the state in ec_slave[]
        ec_readstate();
        for (int cnt = 1; cnt <= ec_slavecount; ++cnt)
        {
            printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d, StatusCode=0x%4.4x : %s \n",
                    cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                    ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc,
                    ec_slave[cnt].ALstatuscode, ec_ALstatuscode2string(ec_slave[cnt].ALstatuscode));
        }
    }

    void CEcatMaster::close_master()
    {
        ec_close();
    }

}  // namespace ecat
