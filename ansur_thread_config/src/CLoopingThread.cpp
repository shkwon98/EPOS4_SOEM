#include "CLoopingThread.hpp"

CLoopingThread::CLoopingThread()
{
    m_isActive = false;
}
CLoopingThread::~CLoopingThread()
{
    if (m_thread.joinable())
    {
        this->loopStop();
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

void CLoopingThread::rtLoopSet(int64_t period)
{
    struct sched_attr attr;
    attr.size = sizeof(attr);
    sched_rr(&attr, 10, 0);

    clock_gettime(CLOCK_MONOTONIC, &real_time);
    int ht = (real_time.tv_nsec / 1000000) + 1; // round to nearest ms
    real_time.tv_nsec = ht * 1000000;
    toff = 0;
    last_time = real_time;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        std::cout << "mlockall failed: %m\n";
        pthread_cancel(pthread_self());
    }

    m_period = period;
}


bool CLoopingThread::loopStart()
{
    m_isActive = true;
    m_thread = std::thread([&]()
    {
        while (m_isActive)
        {
            this->task();
        }
    });

    return m_isActive;
}

bool CLoopingThread::rtLoopStart(int64_t period)
{
    m_isActive = true;
    this->rtLoopSet(period);

    m_thread = std::thread([&]()
    {
        while (m_isActive)
        {
            add_timespec(&real_time, m_period + toff);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &real_time, NULL);

            // LOOP TIME MEASUREMENT
            clock_gettime(CLOCK_MONOTONIC, &real_time);
            loop_time = (real_time.tv_sec - last_time.tv_sec) * 1e03 + (real_time.tv_nsec - last_time.tv_nsec) * 1e-6;
            last_time = real_time;

            task();

            // TASK TIME MEASUREMENT
            clock_gettime(CLOCK_MONOTONIC, &real_time);
            task_time = (real_time.tv_sec - last_time.tv_sec) * 1e03 + (real_time.tv_nsec - last_time.tv_nsec) * 1e-6;

            printf("[Control Thread] loop_time: %1.2f ms, task_time: %1.2f ms   \r", loop_time, task_time);
            fflush(stdout);
        }
    });

    return m_isActive;
}


bool CLoopingThread::loopStop()
{
    m_isActive = false;
    m_thread.join();

    return m_isActive;
}
bool CLoopingThread::rtLoopStop()
{
    return loopStop();
}
