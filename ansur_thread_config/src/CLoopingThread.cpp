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

bool CLoopingThread::loopStop()
{
    m_isActive = false;
    m_thread.join();
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
            add_timespec(&ts, m_addtime);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

            // LOOP TIME MEASUREMENT //
            clock_gettime(CLOCK_MONOTONIC, &real_time);
            loop_time = (real_time.tv_sec - last_time.tv_sec) * 1e03 + (real_time.tv_nsec - last_time.tv_nsec) * 1e-6;
            last_time = real_time;

            task();

            // TASK TIME MEASUREMENT //
            clock_gettime(CLOCK_MONOTONIC, &real_time);
            task_time = (real_time.tv_sec - last_time.tv_sec) * 1e03 + (real_time.tv_nsec - last_time.tv_nsec) * 1e-6;

            // UNCOMMENT TO PRINT THE TIME MEASUREMENTS //
            printf("[Control Thread] loop_time: %1.2f ms, task_time: %1.2f ms   \r", loop_time, task_time);
            fflush(stdout);
        }
    });

    return m_isActive;
}

bool CLoopingThread::rtLoopStop()
{
    return loopStop();
}
