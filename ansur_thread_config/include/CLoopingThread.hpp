#ifndef CLOOPINGTHREAD_HPP
#define CLOOPINGTHREAD_HPP

#include <iostream>
#include <thread>
#include "scheduling.hpp"

#define NSEC_PER_SEC 1000000000

class CLoopingThread
{
public:
    CLoopingThread();
    virtual ~CLoopingThread();

    bool loopStart();
    bool rtLoopStart(int64_t period);

    bool loopStop();
    bool rtLoopStop();


protected:
    virtual bool task() = 0;
    int64_t m_period;
    int64_t toff;


private:
    std::thread m_thread;
    bool m_isActive;
    struct timespec real_time, last_time;
    double task_time, loop_time;

    void add_timespec(struct timespec *ts, int64_t addtime);
    void rtLoopSet(int64_t period);
};

#endif // CLOOPINGTHREAD_HPP
