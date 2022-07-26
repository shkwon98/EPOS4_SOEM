#ifndef CLOOPINGTHREAD_HPP
#define CLOOPINGTHREAD_HPP

#include <iostream>
#include <thread>
#include "scheduling.hpp"

#define NSEC_PER_SEC 1000000000
#define CONTROL_PERIOD 1000.0 // us

class CLoopingThread
{
public:
    CLoopingThread();
    virtual ~CLoopingThread();

    void add_timespec(struct timespec *ts, int64_t addtime);
    void rtLoopSet(int64_t cycletime);
    bool rtLoopStart(int64_t cycletime);
    bool loopStart();
    bool stop();
    bool isActive();

protected:
    virtual bool task() = 0;
    int64_t cycletime;
    int64_t toff = 0;

private:
    bool m_isActive;
    std::thread m_thread;

    struct timespec next_time;
    struct timespec real_time, last_time;
    double run_time;

};

#endif // CLOOPINGTHREAD_HPP
