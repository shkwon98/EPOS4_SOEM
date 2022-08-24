#include "CLoopingThread.hpp"
#include "csvWrite.hpp"
#include <vector>

CLoopingThread::CLoopingThread() {}
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
        // std::vector<double> vec1;
        // std::vector<double> vec2;
        // std::vector<double> vec3;
        // vec1.reserve(70000);
        // vec2.reserve(70000);
        // vec3.reserve(70000);
        // double i = 0;

        while (m_isActive)
        {
            add_timespec(&ts, m_addtime);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

            /******  LOOP TIME MEASUREMENT  ******/
            clock_gettime(CLOCK_MONOTONIC, &real_time);
            loop_time = (real_time.tv_sec - last_time.tv_sec) * 1e03 + (real_time.tv_nsec - last_time.tv_nsec) * 1e-6;
            last_time = real_time;

            /** RT TASK **/
            task();

            /******  TASK TIME MEASUREMENT  ******/
            clock_gettime(CLOCK_MONOTONIC, &real_time);
            task_time = (real_time.tv_sec - last_time.tv_sec) * 1e03 + (real_time.tv_nsec - last_time.tv_nsec) * 1e-6;

            /******  UNCOMMENT TO PRINT THE TIME MEASUREMENTS  ******/
            // printf("[Control Thread] loop_time: %1.2f ms, task_time: %1.2f ms   \r", loop_time, task_time);
            // fflush(stdout);


            // vec1.push_back(i++);
            // vec2.push_back(loop_time);
            // vec3.push_back(task_time);
        }

        // std::vector<std::vector<double>> vals = { vec1, vec2, vec3 };
        // write_csv("/home/pi/workspace/1.Ansur_SOEM/log/1ms_loop_3.csv", vals);
    });

    return m_isActive;
}

bool CLoopingThread::rtLoopStop()
{
    return loopStop();
}
