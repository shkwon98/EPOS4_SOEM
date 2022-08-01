#ifndef CCONTROLTHREAD_HPP
#define CCONTROLTHREAD_HPP

#include <iostream>
#include <math.h>
#include "ecatDef.hpp"
#include "socketDef.hpp"
#include "CUdpPacket.hpp"
#include "CLoopingThread.hpp"


class CControlThread: public CLoopingThread
{
public:
    CControlThread();
    ~CControlThread();

protected:
    virtual bool task() override;

private:
    int64_t toff = 0;

    CUdpPacket* udpPacket;

    unsigned int tick = 0;
    double curPos = 0.0;
    double tarPos = 0.0;
    bool bEndFlag = true;
    double endCnt = 0;

    int started[EPOS4_NUM] = { 0 };

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

    double sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now);
};

#endif // CCONTROLTHREAD_HPP
