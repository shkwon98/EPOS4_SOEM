#ifndef __CPDOMAPPING_HPP__
#define __CPDOMAPPING_HPP__

#include <stdio.h>
#include <stdlib.h>
#include "ethercat.h"


class CPdoMapping
{
public:
    static int mapMotorPDOs(uint16 slaveIdx);
    static int initMotor(uint16 slaveIdx);

private:
    static int wkc;
    static int retVal;

    static int SDO_write8(uint16 slaveIdx, uint16 index, uint8 subindex, uint8 value);
    static int SDO_write16(uint16 slaveIdx, uint16 index, uint8 subindex, uint16 value);
    static int SDO_write32(uint16 slaveIdx, uint16 index, uint8 subindex, uint32 value);
};

#endif  // __CPDOMAPPING_HPP__
