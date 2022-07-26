#ifndef CCONTROLTHREAD_HPP
#define CCONTROLTHREAD_HPP

#include <iostream>
#include <math.h>
#include "CLoopingThread.hpp"
#include "CUdpPacket.hpp"
#include "socketDef.hpp"
#include "CEcatCommand.hpp"

#define NUMOF_EPOS4 1

class CControlThread: public CLoopingThread
{
public:
    CControlThread();
    ~CControlThread();

protected:
    virtual bool task() override;

private:
    CUdpPacket* udpPacket;

    unsigned int tick = 0;
    double curPos = 0.0;
    double tarPos = 0.0;
    bool bEndFlag = true;
    double endCnt = 0;

    int started[NUMOF_EPOS4] = { 0 };

    void ec_sync(int64 refTime, int64 cycleTime, int64 *offsetTime);
    double sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now);
    int servo_enable(uint16 StatusWord, uint16 *ControlWord);
};

#endif // CCONTROLTHREAD_HPP
