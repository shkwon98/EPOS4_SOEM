#include "SOEM_EPOS.h"


EtherCAT_EPOS4::EPOS4::epos4_rx *rxPDO[EPOS_NUM];  
EtherCAT_EPOS4::EPOS4::epos4_tx *txPDO[EPOS_NUM];


namespace Conversion {
    
    double  CntPSecToRadPsec(double cntPSec, int cntPRev) {return (double) cntPSec * _2PI / cntPRev;} // [Cnt/s] -> [Rad/s]
    double  RadPsecToMPsec(double radPsec, long lead)          {return (double) radPsec / _2PI * lead;} // [Rad/s] -> [m/s]
    double  mPsecToRadPsec(double mPsec, long lead)            {return (double) _2PI * mPsec / lead;} // [m/s] -> [Rad/s]
    double  RadPsecToCntPsec(double radPSec, int cntPRev) {return (double) cntPRev * radPSec / _2PI;} //[Rad/s] -> [Cnt/s]
    double  DegPsecToRPM(double DegPsec, long gearRatio) {return (double)DegPsec * gearRatio * 60 / 360;} //[Degree/s] -> [RPM]
    double  RPMToDegPsec(double RPM, long gearRatio) {return (double)RPM * 360 / (60 * gearRatio);} // [RPM] -> [degree/s]

    double	CntToRad(long cnt, int cntPRev)		{return (double)cnt * _2PI / cntPRev;} // [Cnt] -> [Rad]
    double	CntToDeg(double  cnt, int cntPRev)		{return (double)cnt * 360.0 / cntPRev;} // [Cnt] -> [degree]
    double  RadToCnt(double rad, int cntPRev) {return cntPRev * rad / _2PI;} // [Rad] -> [Cnt]
    double  DegToCnt(double deg, int cntPRev) {return (double)deg * cntPRev / 360;} // [Degree] -> [Cnt]
    double  RadToPosition(double rad, long lead) {return (double)rad / _2PI * lead;} // [Rad] -> [mm]
    double  PositionToRad(double pos, long lead) {return (double)pos / lead * _2PI;} // [mm] -> [Rad]
    double  PositionToCnt(double pos, int cntPmm) {return (double)pos * cntPmm;} // [mm] -> [Cnt]
    double  CntToPosition(double cnt, int cntPmm) {return (double)cnt / cntPmm;} // [Cnt] -> [mm]

    int     PositionCorrection(int pos, int homePos) {return pos - homePos;} // Consider instant position as 0

};





//////////////////////// Class : SOEM ////////////////////////


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
    }
    while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        printf("EPOS4 : All slaves reached operational state\n");
        // usleep(500000);
        for ( int slave = 1; slave <= ec_slavecount; slave++ )
        {
            rxPDO[slave - 1] = (EtherCAT_EPOS4::EPOS4::epos4_rx *)(ec_slave[slave].outputs);
            txPDO[slave - 1] = (EtherCAT_EPOS4::EPOS4::epos4_tx *)(ec_slave[slave].inputs);
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
void *SOEM::ecatcheck( void *ptr ) // SOEM providing function. Check the EtherCAT communication state, and recover when the error occured.
{
    int slave;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate) )
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
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                    //  printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > 0)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (!ec_slave[slave].state)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(!ec_slave[slave].state)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf(".");
        }
        usleep(250);
    }
}

//////////////////////////////////////////////////////////////




//////////////////////// Class : LOOP ////////////////////////

// Variables //
struct timespec LOOP::ts;
int LOOP::wkcErrorCount = 0;
int LOOP::safetyErrorCount = 0;
int LOOP::loopCounter = 0;
double LOOP::elapsedTime = 0;
RTIME LOOP::timeNow;
int LOOP::shmid;

// Flags //
bool LOOP::etherCATLoop = true;
bool LOOP::etherCATWKC = false;

// Functions //
/** Callback function to Ctrl+c command
*/
void LOOP::signalCallbackHandler(int signum)
{
    printf("\nCaught Ctrl+c %d\n", signum);
    printf("Terminating EtherCAT!!\n");

    etherCATLoop = false;
//   log_statefile();
     // Terminate program
    shmctl(LOOP::shmid, IPC_RMID, 0);
    exit(signum);
}

/** Get current time to LOOP::ts
*/
void LOOP::getTime()
{
    timeNow = rt_timer_read();
}

/** Check time and wait until the period is done. LOOP::loopCounter and LOOP::elapsedTime rises.
*/
void LOOP::checkPeriod()
{
    static u_long previous = timeNow;
    rt_task_wait_period(NULL);
    getTime();
    u_long step = timeNow - previous;
    previous = timeNow;
    // printf("SOEM rt_thread period %li\n", step);

    loopCounter++;
    if (loopCounter == 100000)
    {
        loopCounter = 1000;
    }
}

/** Check time and wait until the period is done. Send/receive PDO and check wkc and decide stop or not.
 * @return                        False if wkc error keep occured
*/
bool LOOP::continueLoop()
{
    checkPeriod();
    ec_send_processdata();
    SOEM::wkc = ec_receive_processdata(EC_TIMEOUTRET);

    // Check working counter
    if (SOEM::wkc >= SOEM::expectedWKC)
    {
        etherCATWKC = true;
        wkcErrorCount = 0;
    }
    else
    {
        etherCATWKC = false;
    }

    // Check if wkc error continuously occured for long time
    if (wkcErrorCount == 5)
    {
        printf("WKC error keep occured! Stop control loop");
        etherCATLoop = false;
    }
    // Check if safety error occured many times
    if (safetyErrorCount == 10)
    {
        printf("Safety error keep occured! Stop control loop");
        etherCATLoop = false;
    }

    // Check Flags to decide continue loop
    if ( etherCATLoop && etherCATWKC )
    {
        return true;
    }
    else if ( etherCATLoop && !etherCATWKC )
    {
        if (loopCounter % 200 == 0)
        {
            printf("EtherCAT working counter is not correct. Now fixing...\n");
            wkcErrorCount++;
        }
        return true;
    }
    else
    {
        return false;
    }
}



//////////////////////////////////////////////////////////////




//////////////////////// Class : CONTROL ////////////////////////

// Variables //
double CONTROL::homingSensor[EPOS_NUM];
Eigen::Vector3d CONTROL::Haptic;

// Flags //
bool CONTROL::PPMFlag = false;
bool CONTROL::QSFlag[EPOS_NUM] = { false };



// Functions //
/** Command control word according to the read statusword
 * @return                        true if motor is operational
*/
bool CONTROL::controlWordGenerate(uint slaveID)
{
    uint16_t statusWord = txPDO[slaveID - 1] -> statusWord;
    uint16_t* controlWord = &(rxPDO[slaveID - 1] -> controlWord);

    if ( rxPDO[slaveID - 1] -> modeOfOperation == ProfilePositionmode )
    {
        if ( (LOOP::loopCounter % 2) == 0 )
        {
            PPMFlag = true;
        }
    }

    if (QSFlag[slaveID - 1] == TRUE)
    {
        *controlWord = QK_STOP;
        return false;
    }
    else
    {
        if (!(statusWord & (1 << OPERATION_ENABLE_BIT))) //4
        {
            if (!(statusWord & (1 << SWITCHED_ON_BIT))) //2
            {
                if (!(statusWord & (1 << READY_TO_SWITCH_ON_BIT))) //1
                {
                    if (statusWord & (1 << FAULT_BIT)) //8
                    {
                        *controlWord = 0x80;                       
                        return false;
                    }
                    else
                    {
                        *controlWord = CW_SHUTDOWN;
                        return false;                    
                    }
                }
                else
                {
                    *controlWord = CW_SWITCHON;
                    return true;                
                }
            }
            else if(PPMFlag == true)
            {
                *controlWord = PPM_OP;
                if (statusWord & 0x1000)
                {
                    PPMFlag = FALSE;
                }
                return true;
            }
            else
            {
                *controlWord = CW_ENABLEOP;
                return true;            
            }
        }
        else if(PPMFlag == TRUE)
        {
            *controlWord = PPM_OP;
            if (statusWord & 0x1000)
            {
                PPMFlag = FALSE;
            }
            return true;
        }
        else
        {
            *controlWord = CW_ENABLEOP;
            return true;        
        }    
        *controlWord = 0;
        return false;
    }


}

/////////////////////////////////////////////////////////////////




/////////////////////// Class : EPOS4 ///////////////////////
int EPOS4::slaveCount = 0;
int EPOS4::homingCount = 0;
int EPOS4::jointLimit[EPOS_NUM] = {0,};
int EPOS4::jointLimitCount = 0;


// Locally used variables (in functions only) //
char slaveName[EPOS_NUM][16] = {"ENDO_A_M1", "ENDO_A_M2", "ENDO_A_M3", "ENDO_A_M4", "RCM_B_M1", "RCM_B_M2", "CARRI_A_M1", "CARRI_A_M2", "CARRI_B_M1", "CARRI_B_M2", "MAXPOS"};
char serialNum[EPOS_NUM][16] = { ENDO_A_M1, ENDO_A_M2, ENDO_A_M3, ENDO_A_M4, RCM_B_M1, RCM_B_M2, CARRI_A_M1, CARRI_A_M2, CARRI_B_M1, CARRI_B_M2, "0x12002097" };

char hstr[1024];
char usdo[128];


// Functions //
/** Generate EPOS4 Object
 * 
 * @param[in] serialNumber       Slave serial number
 * @param[in] direction          Motor rotation direction. CCW/CW or UP/DOWN based on joint z-axis.
 * @param[in] jointType          REVOLUTE or PRISMATIC
 * @param[in] gearRatio          If encoder position is on gear, put ENCODER_ON_GEAR.
 * @param[in] lead               Screw lead [mm] per [revolute]. If joint type is REVOLUTE, put REVOLUTE.
 * @param[in] resolution         Encoder resolution. Total counts per turn.
*/
EPOS4::EPOS4(char* serialNumber, int direction, int jointType, long gearRatio, double lead, long resolution)
{
    this->mSlaveID = 200;
    this->mSerialNumber = serialNumber;
    this->mDirection = direction;
    this->mJointType = jointType;
    this->mGearRatio = gearRatio;
    this->mLead = lead;
    this->mResolution = resolution;
    this->mConnected = false;
    this->mHoming = false;
    this->mSetHome = false;

    this->mMaxPosDeg = 0;
    this->mMinPosDeg = 0;
    this->mMaxPosmm = 0;
    this->mMinPosmm = 0;

    this->mPotentioHome = -1;
    this->mHomePosition = 0;

    this->mModeOfOperation = ProfilePositionmode;
    this->mTargetVelocity = 0;
    this->mTargetPosition = 0;

    for (int i = 1; i<=EPOS_NUM; i++)
    {
        if (!strcmp(serialNumber, serialNum[i-1]))
        {
            this->mSlaveName = slaveName[i-1];
            // Is this needed? Check later
            this->mSlaveUniqueID = i;
            printf("Slave %s is generated\n", slaveName[i-1]);
        }
    }


    //For Endo, RCM and Disk-Stage
    this->mCntPer_Rev = direction * gearRatio * resolution;
    //For Z-Stage
    if (jointType == PRISMATIC)
    {
        this->mCntPer_mm = direction * gearRatio * resolution / lead;
    }
}

/** Check if slave is connected
 * 
 * @return                       Slave ID if connected, -1 otherwise.
*/
int EPOS4::checkConnection(void)
{
    int l = sizeof(usdo) - 1;
    uint32_t *u32;

    for (int slaveID = 1; slaveID <= ec_slavecount; slaveID++)
    {
        memset(&usdo, 0, 128);
        ec_SDOread(slaveID, (uint16) 0x1018, (uint8) 0x04, false, &l, &usdo, EC_TIMEOUTRXM);
        u32 = (uint32_t*) &usdo[0];
        sprintf(hstr, "0x%8.8x", *u32);

        if (!strcmp(hstr, this->mSerialNumber))
        {
            this->mSlaveID = slaveID;
            this->mConnected = true;
            printf("Slave %s connected\n", this->mSlaveName);
            slaveCount++;
            return this->mSlaveID;
        }
        else
        {
            this->mSlaveID = -1;
        }
    }
    if (this->mSlaveID == -1)
    {
        printf("Slave %s Not connected!!!\n", this->mSlaveName);
        return -1;
    }
}

/** Map PDO objects to the slave
 * 
 * @param[in] rxPDO              Recieve(send to slave) PDO objects. Given by array of { Index + SubIndex + DataSize }, f.e. 0x60400010 for Controlword
 * @param[in] txPDO              Transmit(sent from slave) PDO objects. Given by array of { Index + SubIndex + DataSize }, f.e. 0x60410010 for Statusword
*/
void EPOS4::mapPDO(uint32_t* rxPDO, uint32_t*txPDO, uint8_t rxPDOSize, uint8_t txPDOSize)
{
    if (this->mConnected == true)
    {
        int SDOTrial = 0;

        int retvalSYNC = 0;
        int retvalPDO = 0;

        uint8_t zero = 0x00;

        uint16_t map1c12[2] = {0x0001, 0x1601};
        uint16_t map1c13[2] = {0x0001, 0x1a01};

        while(SDOTrial < 5)
        {
            SDOTrial = 0;
            
            retvalSYNC = 0;
            retvalPDO = 0;

            retvalSYNC += ec_SDOwrite(this->mSlaveID, 0x1c12, 0x00, false, sizeof(zero), &zero, EC_TIMEOUTSAFE*4);
            retvalSYNC += ec_SDOwrite(this->mSlaveID, 0x1c13, 0x00, false, sizeof(zero), &zero, EC_TIMEOUTSAFE*4);

            //rxPDO mapping
            retvalPDO += ec_SDOwrite(this->mSlaveID, 0x1601, 0x00, false, sizeof(zero), &zero, EC_TIMEOUTSAFE);
            for (int i=0; i<rxPDOSize; i++)
            {
                retvalPDO += ec_SDOwrite(this->mSlaveID, 0x1601, 0x01+i, FALSE, sizeof(rxPDO[i]), &rxPDO[i], EC_TIMEOUTSAFE);
            }
            retvalPDO += ec_SDOwrite(this->mSlaveID, 0x1601, 0x00, FALSE, sizeof(rxPDOSize), &rxPDOSize, EC_TIMEOUTSAFE);
            //txPDO mapping
            retvalPDO += ec_SDOwrite(this->mSlaveID, 0x1a01, 0x00, FALSE, sizeof(zero), &zero, EC_TIMEOUTSAFE);
            for (int i=0; i<txPDOSize; i++)
            {
                retvalPDO += ec_SDOwrite(this->mSlaveID, 0x1a01, 0x01+i, FALSE, sizeof(txPDO[i]), &txPDO[i], EC_TIMEOUTSAFE);
            }
            retvalPDO += ec_SDOwrite(this->mSlaveID, 0x1a01, 0x00, FALSE, sizeof(txPDOSize), &txPDOSize, EC_TIMEOUTSAFE);
            

            retvalSYNC += ec_SDOwrite(this->mSlaveID, 0x1c12, 0x00, TRUE, sizeof(map1c12), &map1c12, EC_TIMEOUTSAFE*4);

            retvalSYNC += ec_SDOwrite(this->mSlaveID, 0x1c13, 0x00, TRUE, sizeof(map1c13), &map1c13, EC_TIMEOUTSAFE*4);


            //Add code that reads subindex and check if the rxpdosize == read subindex && txpdosize == read subindex. That's how check if the mapping was succeed.
            if (retvalSYNC == 4 && retvalPDO == (rxPDOSize + txPDOSize + 4))
            {
                printf("%s : PDO Mapping succeed\n", mSlaveName);
                return;
            }
            SDOTrial++;
        }
        printf("%s : PDO Mapping failed. Exit process\n", mSlaveName);
        exit(1);
    }

}

/** Set motor position limit from Home. [Deg] for REVOLUTE joint, [mm] for PRISMATIC joint. Save max/min encoder counts in member variables mMax/MinPosCnt.
 * 
 * @param[in] maxPos             Maximum degree/mm limit from home position
 * @param[in] minPos             Minimum degree/mm limit from home position
 * @param[in] velocityDecimal    Define velocity unit.  ex) 0 -> rpm / 3 -> 0.001rpm
*/
void EPOS4::setLimit(DEGREE maxPos, DEGREE minPos, int velocityDecimal)
{
    uint32_t SIUnitVelocity;

    switch(velocityDecimal)
    {
        case 0:
            SIUnitVelocity = 0x00B44700;
            this->mVelocityDecimal = 0;
            break;
        case 1:
            SIUnitVelocity = 0xFFB44700;
            this->mVelocityDecimal = 1;
            break;
        case 2:
            SIUnitVelocity = 0xFEB44700;
            this->mVelocityDecimal = 2;
            break;
        case 3:
            SIUnitVelocity = 0xFDB44700;
            this->mVelocityDecimal = 3;
            break;
        case 4:
            SIUnitVelocity = 0xFCB44700;
            this->mVelocityDecimal = 4;
            break;
        case 5:
            SIUnitVelocity = 0xFBB44700;
            this->mVelocityDecimal = 5;
            break;
        case 6:
            SIUnitVelocity = 0xFAB44700;
            this->mVelocityDecimal = 6;
            break;
        default:
            SIUnitVelocity = 0xFDB44700;
            this->mVelocityDecimal = 3;
            break;
    }

    if (this->mJointType == REVOLUTE)
    {
        this->mMaxPosDeg = maxPos;
        this->mMinPosDeg = minPos;
        if (this->mDirection == CCW)
        {
            this->mMaxPosCnt = Conversion::DegToCnt(maxPos, this->mCntPer_Rev);
            this->mMinPosCnt = Conversion::DegToCnt(minPos, this->mCntPer_Rev);
        }
        else if (this->mDirection == CW)
        {
            this->mMaxPosCnt = Conversion::DegToCnt(minPos, this->mCntPer_Rev);
            this->mMinPosCnt = Conversion::DegToCnt(maxPos, this->mCntPer_Rev);
        }
    }
    else if (this->mJointType == PRISMATIC)
    {
        this->mMaxPosmm = maxPos;
        this->mMinPosmm = minPos;
        if (this->mDirection == UP)
        {
            this->mMaxPosCnt = Conversion::PositionToCnt(maxPos, this->mCntPer_mm);
            this->mMinPosCnt = Conversion::PositionToCnt(minPos, this->mCntPer_mm);
        }
        else if (this->mDirection == DOWN)
        {
            this->mMaxPosCnt = Conversion::PositionToCnt(minPos, this->mCntPer_mm);
            this->mMinPosCnt = Conversion::PositionToCnt(maxPos, this->mCntPer_mm);
        }
    }

    if (this->mConnected)
    {
        int SDOTrial = 0;
        while(SDOTrial < 5)
        {
            SDOTrial = 0;
            int retval = ec_SDOwrite(this->mSlaveID, 0x60a9, 0x00, false, sizeof(SIUnitVelocity), &SIUnitVelocity, EC_TIMEOUTSAFE*4);
            if (retval == 1){return;}
            SDOTrial++;
        }
        printf("Velocity unit setting failed. Please reboot Robot control box.\n");
        exit(1);
    }
}

/** Check current position and turn QuickStop flag on when cpos exceeds limit
*/
void EPOS4::checkSafety()
{
    int cPos = this->mPositionActualValue - this->mHomePosition;

    if ( cPos > (this->mMaxPosCnt + this->mMaxPosCnt/10) )
    {
        CONTROL::QSFlag[this->mSlaveID - 1] = true;
        if ( (CONTROL::QSFlag[this->mSlaveID - 1] == true) && (this->mTargetPosition <= this->mMaxPosCnt) )
        {
            CONTROL::QSFlag[this->mSlaveID - 1] = false;
        }

        if ( this->mHoming == true && LOOP::loopCounter % 10 == 0)
        {
            printf("Qs\n");
            LOOP::safetyErrorCount++;
        }
    }
    else if ( cPos < (this->mMinPosCnt - abs(this->mMinPosCnt)/10) )
    {
        CONTROL::QSFlag[this->mSlaveID - 1] = true;
        if ( (CONTROL::QSFlag[this->mSlaveID - 1] == true) && (this->mTargetPosition >= this->mMinPosCnt) )
        {
            CONTROL::QSFlag[this->mSlaveID - 1] = false;
        }
        
        if ( this->mHoming == true && LOOP::loopCounter % 10 == 0)
        {
            printf("Qs\n");
            LOOP::safetyErrorCount++;
        }
    }
}

/** Write target values in member variables to txPDO
*/
void EPOS4::writeTarget()
{
    if ( this->mConnected == true )
    {
        checkSafety();
        if ( CONTROL::controlWordGenerate(this->mSlaveID) )
        {
            rxPDO[this->mSlaveID - 1]->modeOfOperation = this->mModeOfOperation;

            // Check if homing finished
            if (this->mHoming == false)
            {
                rxPDO[this->mSlaveID - 1]->targetVelocity = this->mTargetVelocity;
                rxPDO[this->mSlaveID - 1]->targetPosition = this->mHomePosition;
                rxPDO[this->mSlaveID - 1]->profileVelocity = this->mHomingVelocity;
                rxPDO[this->mSlaveID - 1]->profileAcceleration = 10000;
                rxPDO[this->mSlaveID - 1]->profileDeceleration = 10000;
            }
            else
            {
                // Check if dPos exceeds limit
                if (this->mTargetPosition >= this->mMaxPosCnt)
                {
                    printf("%s Max\n", this->mSlaveName);
                    rxPDO[this->mSlaveID - 1]->targetPosition = this->mMaxPosCnt + this->mHomePosition;
                    jointLimit[this->mSlaveID - 1] = 1;
                }
                else if (this->mTargetPosition <= this->mMinPosCnt)
                {
                    printf("%s Min\n", this->mSlaveName);
                    rxPDO[this->mSlaveID - 1]->targetPosition = this->mMinPosCnt + this->mHomePosition;
                    jointLimit[this->mSlaveID - 1] = 1;
                }
                else
                {
                    rxPDO[this->mSlaveID - 1]->targetPosition = this->mTargetPosition + this->mHomePosition;
                    jointLimit[this->mSlaveID - 1] = 0;
                }

                rxPDO[this->mSlaveID - 1]->profileVelocity = this->mProfileVelocity;
                rxPDO[this->mSlaveID - 1]->profileAcceleration = this->mProfileAcceleration;
                rxPDO[this->mSlaveID - 1]->profileDeceleration = this->mProfileDeceleration;
            }
        }
    }

}

/** Read txPDO and save it to the object member variables
*/
void EPOS4::readState()
{
    if ( this->mConnected == true)
    {
        this->mStatusWord = txPDO[this->mSlaveID - 1]->statusWord;
        this->mModeOfOperationDisplay = txPDO[this->mSlaveID - 1]->modeOfOperationDisplay;
        this->mPositionActualValue = txPDO[this->mSlaveID - 1]->positionActualValue;
        this->mVelocityActualValue = txPDO[this->mSlaveID - 1]->velocityActualValue;
        this->mTorqueActualValue = txPDO[this->mSlaveID - 1]->torqueActualValue;
        this->mPositionDemandValue = txPDO[this->mSlaveID - 1]->positionDemandValue;
        getJointValue();
    }
}

/** Correct homing sensor value to make home position value 1.025 (middle value)
 * @return                       Corrected sensor value
*/
float EPOS4::correctHomingSensor()
{
    float homeOffset = this->mPotentioHome - 1.025;
    float corrected;

    if ( homeOffset < 0 )
    {
        if ( CONTROL::homingSensor[this->mSlaveUniqueID - 1] > (2.05 + homeOffset) )
        {
            corrected = CONTROL::homingSensor[this->mSlaveUniqueID - 1] - (2.05 + homeOffset);
        }
        else
        {
            corrected = CONTROL::homingSensor[this->mSlaveUniqueID - 1] - homeOffset;
        }
    }
    else
    {
        if ( CONTROL::homingSensor[this->mSlaveUniqueID - 1] < homeOffset )
        {
            corrected = 2.05 - (homeOffset - CONTROL::homingSensor[this->mSlaveUniqueID - 1]);
        }
        else
        {
            corrected = CONTROL::homingSensor[this->mSlaveUniqueID - 1] - homeOffset;
        }
    }

    return corrected;
}

/** Homing function
 * @param[in] homingVel          Homing velocity in [RPM] unit. Meaningless for incremental encoder.
 * @return                       True if homing is finished
*/
void EPOS4::doHoming(uint32_t homingVel)
{
    if ( this->mConnected == true)
    {
        if ( this->mHoming == false)
        {
            if ( this->mSetHome == true)
            {
                if ( !(this->mPotentioHome == -1) )
                {
                    this->mModeOfOperation = CyclicSynchronousVelocitymode;
                    this->mTargetVelocity = (1.025 - correctHomingSensor()) * 50 / (fabs(1.025 - correctHomingSensor()) + 0.025);
                    if (fabs(1.025 - correctHomingSensor()) < 0.0005)
                    {
                        this->mModeOfOperation = ProfilePositionmode;
                        this->mHomePosition = txPDO[this->mSlaveID - 1]->positionActualValue;
                        this->mHoming = true;
                        homingCount++;
                    }
                    // Check if teensy if off
                    // if (LOOP::loopCounter = 100 && )
                }
                else
                {

                    this->mModeOfOperation = ProfilePositionmode;
                    this->mHomingVelocity = homingVel * pow(10, this->mVelocityDecimal);
                    // Activate if ONMOTOR Encoder type acts 1rpm -> 6 deg/sec
                    // this->mHomingVelocity = Conversion::DegPsecToRPM(homingVel, 1);
                    if (abs(this->mPositionActualValue - this->mHomePosition) < 30)
                    {
                        this->mHoming = true;
                        homingCount++;
                    }
                }
            }
        }
    }
}

/** Set home position. Must be called between going operational and control loop.
 * @param[in] encoderType        ABSOLUTE or INCREMENTAL
 * @param[in] homePosition       [Absolute encoder] -> encoder value. [Incremental encoder] -> homing sensor(potentiometer) value.
*/
void EPOS4::setHomePosition(int encoderType, double homePosition)
{
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    if ( this->mConnected == true )
    {
        rxPDO[this->mSlaveID - 1]->targetVelocity = 0;
        rxPDO[this->mSlaveID - 1]->targetPosition = txPDO[this->mSlaveID - 1]->positionActualValue;
        rxPDO[this->mSlaveID - 1]->profileVelocity = 0;
    }

    if ( encoderType == ABSOLUTE )
    {
        this->mHomePosition = homePosition;
    }
    else if( encoderType == INCREMENTAL )
    {
        this->mPotentioHome = homePosition;
        printf("%f\n", this->mPotentioHome);
    }
    else
    {
        this->mHomePosition = txPDO[this->mSlaveID - 1]->positionActualValue;
    }
    this->mSetHome = true;
}


/** Save current joint value in rad/mm
*/
void EPOS4::getJointValue()
{
    if (mJointType == REVOLUTE)
    {
        this->mJointValue = Conversion::CntToRad(this->mPositionActualValue - this->mHomePosition, this->mCntPer_Rev);
    }
    else if (mJointType == PRISMATIC)
    {
        this->mJointValue = Conversion::CntToPosition(this->mPositionActualValue - this->mHomePosition, this->mCntPer_mm);
    }
}


/** Make velocity profile. Changes the member variables mProfileVelocity and mProfileAcceleration/Deceleration
 * @param[in] deltaJoint         Joint displacement for next step. Rad for REVOLUTE, mm for PRISMATIC
 * @param[in] periodMs           Step period in ms
 * @param[in] t1                 Acceleration time in ms
 * @param[in] t2                 Deceleration time in ms
*/
void EPOS4::makeVelocityProfile(double deltaJoint, double periodMs, double t1, double t2)
{
    int Inc = 0; // Increment [Cnt]
    
    switch(this->mJointType)
    {
        case 0:
            Inc = Conversion::RadToCnt(deltaJoint, this->mCntPer_Rev);
            this->mProfileVelocity = abs(120000 * Inc / ((2 * periodMs - t1 - t2 - 2) * this->mCntPer_Rev) * pow(10, this->mVelocityDecimal));
            this->mProfileAcceleration = abs(this->mProfileVelocity / t1 * 1000);
            this->mProfileDeceleration = abs(this->mProfileVelocity / t2 * 1000);
            break;
        case 1:
            Inc = Conversion::PositionToCnt(deltaJoint, this->mCntPer_mm);
            this->mProfileVelocity = abs(120000 * Inc / ((2 * periodMs - t1 - t2 - 2) * this->mCntPer_Rev) * pow(10, this->mVelocityDecimal));
            this->mProfileAcceleration = abs(this->mProfileVelocity / t1 * 1000);
            this->mProfileDeceleration = abs(this->mProfileVelocity / t2 * 1000);
            break;
        default:
            this->mProfileVelocity = 0;
            break;
    }

    
}


/** Converts target joint to Cnt and saves it to member var mTargetPosition
 * @param[in] targetJoint        Target Joint value. (Revolute : rad / Prismatic : mm)
*/
void EPOS4::convertJointToCnt(double targetJoint)
{
    switch(this->mJointType)
    {
        case 0:
            this->mTargetPosition = Conversion::RadToCnt(targetJoint, this->mCntPer_Rev);
            break;
        case 1:
            this->mTargetPosition = Conversion::PositionToCnt(targetJoint, this->mCntPer_mm);
            break;
    }
}



/** Chek if homing finished for all slaves. Also, sum the jointLimit
 *  @return                       True if homing is finished for all slaves
*/
bool EPOS4::checkHoming()
{
    // After, it should be divided. Doesn't fit to this func.
    jointLimitCount = 0;
    for (int i=0; i<slaveCount; i++)
    {
        jointLimitCount += jointLimit[i];
    }

    if (homingCount == slaveCount)
    {
        return true;
    }
    else {return false;}
}



/////////////////////////////////////////////////////////////
