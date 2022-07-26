#include "CLoopingThread.hpp"

CLoopingThread::CLoopingThread()
{
    m_isActive = false;
}

CLoopingThread::~CLoopingThread()
{
    if (m_thread.joinable())
    {
        stop();
    }
}

void CLoopingThread::add_timespec(struct timespec *ts, int64_t addtime)
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

void CLoopingThread::rtLoopSet(int64_t cycletime)
{
    struct sched_attr attr;
    attr.size = sizeof(attr);
    sched_rr(&attr, 40, 0);

    this->cycletime = cycletime;
}

bool CLoopingThread::rtLoopStart(int64_t cycletime)
{
    m_isActive = true;
    m_thread = std::thread([&]()
    {
        rtLoopSet(cycletime);

        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        {
            std::cout << "mlockall failed: %m\n";
            pthread_cancel(pthread_self());
        }

        while (m_isActive)
        {
            add_timespec(&next_time, cycletime + toff);

            bool bRunTask = task();

            // printf("control_thread cycle time:%.4f      \r", (next_time.tv_nsec - last_time.tv_nsec) / 1000000.0);
            // fflush(stdout);
            next_time.tv_sec += (next_time.tv_nsec + cycletime) / 1e9;
            next_time.tv_nsec = (int)(next_time.tv_nsec + cycletime) % (int)1e9;
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

            // if (bRunTask)
            {
                // TASK TIME MEASUREMENT
                clock_gettime(CLOCK_MONOTONIC, &real_time);
                run_time = (real_time.tv_sec + real_time.tv_nsec * 1e-9) - (last_time.tv_sec + last_time.tv_nsec * 1e-9);
                last_time = real_time;
                printf("control_thread run time:%.4f         \r", run_time);
                fflush(stdout);
            };
        }
    });
    return false;
}

bool CLoopingThread::loopStart()
{
    m_isActive = true;
    m_thread = std::thread([&]()
    {
        while (m_isActive)
        {
            task();
        }
    });

    return isActive();
}

bool CLoopingThread::stop()
{
    m_isActive = false;
    m_thread.join();
    return true;
}

bool CLoopingThread::isActive()
{
    return m_isActive;
}
