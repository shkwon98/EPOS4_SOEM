#ifndef SOEM_HPP
#define SOEM_HPP

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <mutex>
#include "ethercat.h"
#include "ecatDef.hpp"

#define EC_TIMEOUTMON 500

class SOEM
{
public:
    static bool inOP; // True if all slaves become Operational(CANOpen) by SOEM::goingOperational()

    static void initializeEtherCAT(const char* ifname);
    static void goingSafeOP(int (*setup)(uint16 slaveIdx));
    static void goingOperational();
    static void mapIOStructs(PDO_STRUCT servo[], int num_of_servos);
    static void terminateEtherCAT();

    static void ec_sync(int64 refTime, int64 cycleTime, int64 *offsetTime);
    static void *ecatSync();
    static void *ecatCheck();

private:
    static char IOmap[4096]; // Input output map(buffer)
    static int chk;
    static int expectedWKC; // (2*write + 1*read) * slave number
    static int wkc;
    static uint8 currentgroup;
};

#endif // SOEM_HPP
