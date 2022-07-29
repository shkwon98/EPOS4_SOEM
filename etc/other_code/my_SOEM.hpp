#ifndef MY_SOEM_HPP
#define MY_SOEM_HPP

#include <iostream>
#include <stdio.h>
#include <unistd.h>
// #include <stdlib.h>
#include "ethercat.h"

#define EC_TIMEOUTMON 500
#define EPOS_NUM 11

///////////////// PDO Setting /////////////////
#pragma pack(push,1)
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

#endif // MY_SOEM_HPP
