


#ifndef __ECXENOMAI_H
#define __ECXENOMAI_H

#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <chrono>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <thread>
#include <pthread.h>
#include <math.h>
#include <fstream>
#include <term.h>
#include <termios.h>
#include <inttypes.h>
#include <Eigen/Dense>
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

#include <sys/ipc.h>
#include <sys/shm.h>

//////// Xenomai ////////
#define printf rt_printf
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
/////////////////////////



#define EC_TIMEOUTMON 500

#define EPOS_NUM 11
#define PERIOD_NS 1000000*5
#define SEC_IN_NSEC 1000000000

#define ushort unsigned short
#define _2PI 6.28318530717959
#define LOG_SIZE 50000
#define _PI 3.1415926535897932384626433
#define DEG2RAD (0.01745329251994329576923690768489)
#define RAD2DEG 1/DEG2RAD
#define GRAVITY 9.80665

#define CCW 1
#define CW -1
#define UP 1
#define DOWN -1
#define POSITIVE 1
#define NEGATIVE -1

#define ENCODER_ON_GEAR 1
#define REVOLUTE 0
#define PRISMATIC 1
#define ABSOLUTE 1
#define INCREMENTAL 2

#define DEGREE double
#define RAD double

///////////////// Driver Serial Number /////////////////
#define ENDO_A_M1  "0x40000407"  // Index 0 EndoM1
#define ENDO_A_M2  "0x40000270"  // Index 1 EndoM2
#define ENDO_A_M3  "0x40003045"  // Index 2 EndoM3
#define ENDO_A_M4  "0x40000404"  // Index 3 EndoM4

#define RCM_B_M1   "0x40000529"  // Index 4
#define RCM_B_M2   "0x40003029"  // Index 5

#define CARRI_A_M1 "0x40003571"  // Index 6
#define CARRI_A_M2 "0x40003051"  // Index 7
#define CARRI_B_M1 "0x40003205"  // Index 8
#define CARRI_B_M2 "0x40000395"  // Index 9
////////////////////////////////////////////////////////

///////////////// PI COM PORT NUMBER /////////////////
#define PI_LEFT_X 101
#define PI_LEFT_Y 102

#define PI_RIGHT_X 111
#define PI_RIGHT_Y 112
//////////////////////////////////////////////////////


enum // Modes of Operation
{
    ProfilePositionmode = 1,
    ProfileVelocitymode = 3,
    Homingmode = 6,
    CyclicSynchronousPositionmode = 8,
    CyclicSynchronousVelocitymode = 9,
    CyclicSynchronousTorquemode = 10,
};

enum // Control word
{
    CW_SHUTDOWN = 6,
    CW_SWITCHON = 7,
    CW_ENABLEOP = 15,
    CW_DISABLEOP = 7,
    PPM_OP = 63,
    QK_STOP = 2,
};

enum // Status word
{
    HOMING_START_BIT = 4,
    FAULT_BIT = 3,
    OPERATION_ENABLE_BIT = 2,
    SWITCHED_ON_BIT = 1,
    READY_TO_SWITCH_ON_BIT = 0,
};




namespace Conversion
{

    double  CntPSecToRadPsec(double cntPSec, int cntPRev);// [Cnt/s] -> [Rad/s]
    double  RadPsecToMPsec(double radPsec, long lead);// [Rad/s] -> [m/s]
    double  mPsecToRadPsec(double mPsec, long lead);// [m/s] -> [Rad/s]
    double  RadPsecToCntPsec(double radPSec, int cntPRev);//[Rad/s] -> [Cnt/s]
    double  DegPsecToRPM(double DegPsec, long gearRatio);//[Degree/s] -> [RPM]
    double  RPMToDegPsec(double RPM, long gearRatio);// [RPM] -> [degree/s]

    double	CntToRad(long cnt, int cntPRev);// [Cnt] -> [Rad]
    double	CntToDeg(double  cnt, int cntPRev);// [Cnt] -> [degree]
    double  RadToCnt(double rad, int cntPRev);// [Rad] -> [Cnt]
    double  DegToCnt(double deg, int cntPRev);// [Degree] -> [Cnt]
    double  RadToPosition(double rad, long lead);// [Rad] -> [mm]
    double  PositionToRad(double pos, long lead);// [mm] -> [Rad]
    double  PositionToCnt(double pos, int cntPmm);// [mm] -> [Cnt]
    double  CntToPosition(double cnt, int cntPmm);// [Cnt] -> [mm]

    int     PositionCorrection(int pos, int homePos);// Consider instant position as 0

    // double	AmpToNm(double ampare, ushort iepos)	{return (ampare * Amp2Torq[iepos]);} // [Amp] -> [Torque]
    // double	NmToAmp(double torque, ushort iepos)	{return (torque * Torq2Amp[iepos]);} // [Torque] -> [Amp]
    // double  TorqToNewton(double torque, ushort iepos) {return (_2PI * 0.8 *(torque - ((9.81*0.5*25.62*0.008*set_gearRatio[iepos])/_2PI*0.8)))/(0.008*set_gearRatio[iepos]);} // [Torque] -> [Force]
    // double  NmToN(double torque, ushort iepos)      {return (_2PI * set_efficiency_lead *torque /(set_lead*set_gearRatio[iepos]));} // [Torque] -> [Force]
    // double  NToNm(double force, ushort iepos)       {return ((force*set_lead*set_gearRatio[iepos])/(_2PI * set_efficiency_lead));} // [Torque] -> [Force]
};


///////////////// PDO Setting /////////////////
#pragma pack(push,1)
namespace EtherCAT_EPOS4
{
    struct EPOS4
    {
        struct epos4_rx
        {
            uint16_t controlWord;
            int8_t modeOfOperation;
            int32_t targetVelocity;
            int32_t targetPosition;
            uint32_t profileVelocity;
            uint32_t profileAcceleration;
            uint32_t profileDeceleration;
        };

        struct epos4_tx
        {
            uint16_t statusWord;
            int8_t modeOfOperationDisplay;
            int32_t positionActualValue;
            int32_t velocityActualValue;
            int16_t torqueActualValue;
            int32_t positionDemandValue;
        };
    };
}
#pragma pack(pop)
///////////////////////////////////////////////






class SOEM
{

public:
    static char IOmap[4096]; // Input output map(buffer)
    static int expectedWKC; // (2*write + 1*read) * slave number
    static volatile int wkc; // Working counter
    static uint8 currentgroup;
    static int chk;

    static boolean inOP; // True if all slaves become Operational(CANOpen) by SOEM::goingOperational()
    static boolean needlf;
    static bool ecatNumOk; // True if found slave number and pre-defined EPOS_NUM matches
    static bool ecatWKCOk; // True if expectedWKC == 3 * slave number


    static bool initializeEtherCAT(const char* ifname);
    static void goingOperational();
    static void *ecatcheck(void *ptr);
};



class LOOP
{
private:
    static bool etherCATLoop; // Motor control while loop flag
    static int wkcErrorCount; // working counter error count

    static struct timespec ts; // Time for period control

public:
    static int loopCounter; // Rises once at a loop. Return to 1,000 if over 100,000.
    static double elapsedTime; // Time elapsed after control started in seconds.

    static bool etherCATWKC; // EtherCAT working counter flag (for control loop)
    static int safetyErrorCount; // Error counter stack. f.e. safety error

    static RTIME timeNow;

    static int shmid; // Shared Memory ID

    static void signalCallbackHandler(int signum);
    static void getTime();
    static void checkPeriod();
    static bool continueLoop();

};




class CONTROL
{
private:
    static bool PPMFlag; // (PPM Mode) New target position will be commanded when this flag is true

public:
    static Eigen::Vector3d Haptic;
    static bool QSFlag[EPOS_NUM]; // Quick stop flag

    // Ros receive //
    static double homingSensor[EPOS_NUM]; // Endo potentiometer values

    static bool controlWordGenerate(uint slaveID);
};



class EPOS4
{

private:
    // uint mSlaveID; // ID for SOEM. The order of EtherCAT connection.
    uint mSlaveUniqueID; // Unique ID that pre-decided for each parts. f.e. 1 for Endo_A_M1 and 5 for RCM_B_M1
    uint mSlaveID; // ID for SOEM. The order of EtherCAT connection.
    char* mSerialNumber; // Motor driver serial number
    char* mSlaveName; // Slave name
    int mDirection; // Rotation or moving direction when encoder tick rising, with respect to Z-axis(of joint). [CCW/UP : 1] [CW/DOWN : -1]
    int mJointType; // 0: revolute  1: prismatic
    long mGearRatio; // Harmonic gear ratio
    long mLead; // Screw lead[mm/rev]
    long mResolution; // Encoder resolution. Counts per turn * 4
    int mVelocityDecimal; // Define velocity unit.  ex) 0 -> rpm / 3 -> 0.001rpm

    bool mConnected; // True if slave is connected in EtherCAT line
    bool mHoming; // True if homing finished

    uint32_t mHomingVelocity; // Homing profile velocity
    double mPotentioHome; // Potentiometer home position for ENDO module. -1 if not using
    bool mSetHome; // Home position setting flag. Method doHoming only works when mSetHome == true.

    static int slaveCount; // Number of connected slaves
    static int homingCount; // Number of slaves finished homing
    static int jointLimit[EPOS_NUM]; // Number of joints met limit

    int mMaxPosCnt; // Maximum pos limit in Cnt
    int mMinPosCnt; // Minimum pos limit in Cnt

public:
    int mHomePosition; // Encoder home position value
    int mCntPer_Rev; // Encoder total counts per rev
    int mCntPer_mm; // Encoder total counts per mm (prismatic)

    DEGREE mMaxPosDeg; // Maximum pos limit in Deg unit
    DEGREE mMinPosDeg; // Minimum pos limit in Deg unit
    double mMaxPosmm; // Maximum pos limit in mm unit
    double mMinPosmm; // Minimum pos limit in mm unit



    double mJointValue; // Joint value from home position. (Rad) or (mm)

    static int jointLimitCount;

    // Write values (rxPDO) //
    int8_t mModeOfOperation;
    int32_t mTargetVelocity; // (Velocity Mode) [RPM]
    int32_t mTargetPosition; // (Position Mode) [Cnt]
    uint32_t mProfileVelocity; // (PPM Mode) Max profile velocity [RPM]
    uint32_t mProfileAcceleration; // (PPM Mode) Profile Acceleration [RPM/s]
    uint32_t mProfileDeceleration; // (PPM Mode) Profile Deceleration [RPM/s]

    // Read values (txPDO) //
    uint16_t mStatusWord;
    int8_t mModeOfOperationDisplay;
    int32_t mPositionActualValue; // [Cnt]
    int32_t mVelocityActualValue; // [RPM]
    int16_t mTorqueActualValue; // [Percent]
    int32_t mPositionDemandValue; // [Cnt]

    EPOS4(char* serialNumber, int direction, int jointType, long gearRatio, double lead, long resolution);

    int checkConnection();
    void mapPDO(uint32_t* rxPDO, uint32_t*txPDO, uint8_t rxPDOSize, uint8_t txPDOSize);
    void setLimit(DEGREE maxPos, DEGREE minPos, int velocityDecimal);
    void checkSafety();
    void writeTarget();
    void readState();
    float correctHomingSensor();
    void doHoming(uint32_t homingVel);
    void setHomePosition(int encoderType, double homePosition);
    void getJointValue();
    void makeVelocityProfile(double deltaJoint, double periodMs, double t1, double t2);
    void convertJointToCnt(double targetJoint);


    static bool checkHoming();

};


#endif