#include "Controller.h"
#include <fstream>
#include <cmath>

namespace sun
{
    Controller::Controller(Meca500 *meca500, float gain)
    {
        this->gain = gain;
        this->meca500 = meca500;
    }

    Controller::~Controller() {}

    void Controller::startThread()
    {
        thread_controller = std::thread(&Controller::position_loop_control, this);
    }

    void Controller::position_loop_control()
    {
        struct sched_attr attr;
        attr.size = sizeof(attr);
        sched_rr(&attr, 30, 0);

        std::cout << "Start real time controller thread\n";

        float error;
        float joint_position_measured;
        float omega[6] = { 0, 0, 0, 0, 0, 0 };
        float joint[6] = { 0, 0, 0, 0, 0, 0 };
        float joint_velocities[6] = { 0, 0, 0, 0, 0, 0 };

        int count = 0;
        int i = 0; //iterazioni
        int sample = 0;

        //test variables
        float data_position_joint[50000];
        float data_error[50000];
        float data_vel[50000];
        float data_measured_vel[50000];
        int64 time_x[50000];

        //control variables
        float theta_0, theta_f = 90;
        float t0;
        float tf = 10;      //Time in ns
        float istant_time = 0;

        float theta_d;
        float b[6] = { 6, -15, 10, 0, 0, 0 };
        float tau = 0;

        meca500->getJoints(joint);
        theta_0 = joint[5];

        //t0 = ec_DCtime;
        t0 = 0;
        while (count < 1)
        {
            //tau = ((float)(ec_DCtime - t0)) / tf;
            //printf("%f\n",tau);
            if (istant_time < tf)
            {
                tau = istant_time / tf;
                istant_time = istant_time + 0.001;
                theta_d = theta_0 + (theta_f - theta_0) * (b[0] * pow(tau, 5) + b[1] * pow(tau, 4) + b[2] * pow(tau, 3));
            }
            else count++;

            //printf("%f\n",theta_d);
            //measured position
            meca500->getJoints(joint);
            joint_position_measured = joint[5];

            error = theta_d - joint_position_measured;

            omega[5] = error * gain;
            meca500->moveJointsVel(omega);

            meca500->getJointsVelocities(joint_velocities);

            if (i < 50000)
            {
                time_x[i] = istant_time;
                data_position_joint[i] = joint_position_measured;
                data_error[i] = error;
                data_vel[i] = omega[5];
                data_measured_vel[i] = joint_velocities[5];
            }
            // if (joint_position_measured > theta_f-0.001  && joint_position_measured < theta_f+0.001)
            //     count++;
            // else
            //     count = 0;
            i++;
            osal_usleep(1000);
        }

        std::ofstream oFile_p("Data_position_Joint6.txt", std::ios_base::out | std::ios_base::trunc);
        if (oFile_p.is_open())
        {
            if (i > 50000)
                i = 50000;
            for (int y = 0; y < i; y++)
            {
                oFile_p << data_position_joint[y] << "\t";
                oFile_p << time_x[y] - time_x[0] << "\n";
            }
            oFile_p.close();
        }

        std::ofstream oFile_e("Data_error.txt", std::ios_base::out | std::ios_base::trunc);
        if (oFile_e.is_open())
        {
            if (i > 50000)
                i = 50000;
            for (int y = 0; y < i; y++)
            {
                oFile_e << data_error[y] << "\t";
                oFile_e << time_x[y] - time_x[0] << "\n";
            }
            oFile_e.close();
        }

        std::ofstream oFile_v("Data_velocities.txt", std::ios_base::out | std::ios_base::trunc);
        if (oFile_v.is_open())
        {
            if (i > 50000)
                i = 50000;
            for (int y = 0; y < i; y++)
            {
                oFile_v << data_vel[y] << "\t";
                oFile_v << data_measured_vel[y] << "\n";
            }
            oFile_v.close();
        }
    }

    void Controller::waitLoop()
    {
        thread_controller.join();
    }
} // namespace sun