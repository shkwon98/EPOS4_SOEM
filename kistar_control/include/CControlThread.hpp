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

    void ec_sync(int64 refTime, int64 cycleTime, int64 *offsetTime)
    {
        static int64 integral = 0;
        int64 delta;

        delta = (refTime) % cycleTime;
        if (delta > (cycleTime / 2)) { delta = delta - cycleTime; }
        if (delta > 0) { integral++; }
        if (delta < 0) { integral--; }
        *offsetTime = -(delta / 100) - (integral / 20);
    }

    void printStatus();
    void sendStatusToGUI();

    void controlWithGUI();
    void controlStandAlone();

    // double curPos = 0.0;
    // double tarPos = 0.0;
    // bool bEndFlag = true;
    // double endCnt = 0;
    // double sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now);

    // bool isMotorTorqueOn = false;
    // void motorTorqueOn();
    // void motorTorqueOff();
};

#endif // CCONTROLTHREAD_HPP
