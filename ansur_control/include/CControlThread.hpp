#ifndef CCONTROLTHREAD_HPP
#define CCONTROLTHREAD_HPP

#include <iostream>
#include <iomanip>
#include <vector>
#include <math.h>
#include <mutex>
#include "ecatDef.hpp"
#include "socketDef.hpp"
#include "CUdpPacket.hpp"
#include "CLoopingThread.hpp"
#include "SOEM.hpp"

using namespace std;

class CControlThread: public CLoopingThread
{
public:
    CControlThread();
    ~CControlThread();

protected:
    void task() final;

private:
    CUdpPacket* udpPacket;

    int64_t toff = 0;

    unsigned int tick = 0;
    double curPos = 0.0;
    double tarPos = 0.0;
    bool bEndFlag = true;
    double endCnt = 0;

    bool isMotorTorqueOn = false;
    void motorTorqueOn();
    void motorTorqueOff();

    void sendMotorState();
    void printMotorStatus();

    double sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now);

    void controlWithGUI();
    void controlTest();

};

#endif // CCONTROLTHREAD_HPP
