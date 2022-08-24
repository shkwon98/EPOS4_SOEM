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

    // bool isMotorTorqueOn = false;
    // void motorTorqueOn();
    // void motorTorqueOff();

    void printStatus();
    void sendStatusToGui();

    double sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now);
    double sinWave(double amplitude, double period, double offset);
    void controlWithGUI();
    void controlTest();

    vector<int> vec1;
    vector<int> vec2;
    vector<int> vec3;
    vector<int> vec4;
    vector<int> vec5;
    vector<int> vec6;
    vector<int> vec7;
    vector<int> vec8;
    vector<int> vec9;
};

#endif // CCONTROLTHREAD_HPP
