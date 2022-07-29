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
    bool loopStop();
    bool rtLoopStart(int64_t period);
    bool rtLoopStop();

protected:
    int64_t m_period;
    int64_t m_addtime;

    virtual bool task() = 0;

private:
    std::thread m_thread;
    bool m_isActive;
    struct timespec ts;
    struct timespec real_time, last_time;
    double task_time, loop_time;

    void add_timespec(struct timespec *ts, int64_t addtime)
    {
        int64_t sec, nsec;

        nsec = addtime % NSEC_PER_SEC;
        sec = (addtime - nsec) / NSEC_PER_SEC;
        ts->tv_sec += sec;
        ts->tv_nsec += nsec;
        if (ts->tv_nsec > NSEC_PER_SEC)
        {
            nsec = ts->tv_nsec % NSEC_PER_SEC;
            ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
            ts->tv_nsec = nsec;
        }
    }
    void rtLoopSet(int64_t period)
    {
        struct sched_attr attr;
        attr.size = sizeof(attr);
        sched_rr(&attr, 10, 0);

        clock_gettime(CLOCK_MONOTONIC, &ts);
        int ht = (ts.tv_nsec / 1000000) + 1; // round to nearest ms
        ts.tv_nsec = ht * 1000000;

        // if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        // {
        //     std::cout << "mlockall failed: %m\n";
        //     pthread_cancel(pthread_self());
        // }

        m_period = period;
        m_addtime = m_period;
        clock_gettime(CLOCK_MONOTONIC, &real_time);
        last_time = real_time;
    }
};

#endif // CLOOPINGTHREAD_HPP
