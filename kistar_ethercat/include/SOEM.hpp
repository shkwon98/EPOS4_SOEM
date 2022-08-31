#ifndef SOEM_HPP
#define SOEM_HPP

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include "ethercat.h"
#include "ecatDef.hpp"

class SOEM
{
public:
    static bool inOP; // True if all slaves become Operational by SOEM::goingOperational()
    static int wkc; // Working Counter

    static void initializeEtherCAT(const char* ifname);
    static void pdoMapping(int (*setup)(uint16 slaveIdx));
    static void goingOperational();
    static void mapIOStructs(PDO_STRUCT servo[], int num_of_servos);
    static void terminateEtherCAT();

    static void *ecatCheck();

private:
    static char IOmap[4096]; // Input output map(buffer)
    static int expectedWKC; // (2*write + 1*read) * slave number
};

#endif // SOEM_HPP
