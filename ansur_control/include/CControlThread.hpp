#ifndef CCONTROLTHREAD_HPP
#define CCONTROLTHREAD_HPP

#include <iostream>
#include <math.h>
#include "CLoopingThread.hpp"
#include "CUdpPacket.hpp"
#include "socketDef.hpp"
#include "CEcatCommand.hpp"

#define NUMOF_EPOS4 1

#define CONTROL_PERIOD_IN_ms  ( 1.0 )  // ms
#define CONTROL_PERIOD_IN_s   ( 1e-03 * CONTROL_PERIOD_IN_ms )
#define CONTROL_PERIOD_IN_us  ( 1e03 * CONTROL_PERIOD_IN_ms )
#define CONTROL_PERIOD_IN_ns  ( 1e06 * CONTROL_PERIOD_IN_ms )

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

    int started[NUMOF_EPOS4] = { 0 };

    void ec_sync(int64 refTime, int64 period, int64 *offsetTime);
    double sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now);
    int servo_enable(uint16 StatusWord, uint16 *ControlWord);
};

#endif // CCONTROLTHREAD_HPP
