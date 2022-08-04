#ifndef CCONTROLTHREAD_HPP
#define CCONTROLTHREAD_HPP

#include <iostream>
#include <math.h>
#include "ecatDef.hpp"
#include "socketDef.hpp"
#include "CUdpPacket.hpp"
#include "CLoopingThread.hpp"
#include "SOEM.hpp"


class CControlThread: public CLoopingThread
{
public:
    CControlThread();
    ~CControlThread();

protected:
    void task() final;

private:
    int64_t toff = 0;

    CUdpPacket* udpPacket;

    unsigned int tick = 0;
    double curPos = 0.0;
    double tarPos = 0.0;
    bool bEndFlag = true;
    double endCnt = 0;

    int started[EPOS4_NUM] = { 0 };

    bool isMotorTorqueOn = false;
    void motorTorqueOn();
    void motorTorqueOff();

    double sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now);

    void sendMotorData();
};

#endif // CCONTROLTHREAD_HPP
