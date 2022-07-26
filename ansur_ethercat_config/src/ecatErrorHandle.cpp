#include "ecatErrorHandle.hpp"

#define EC_TIMEOUTMON 500
#define cycletime 1000000  // ns

static uint8 currentgroup = 0;
static int wkc;
volatile extern int expectedWKC;
volatile extern bool inOP;

extern pthread_mutex_t mtx_IOMap;

OSAL_THREAD_FUNC_RT sync_thread()
{
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    while (inOP)
    {
        pthread_mutex_lock(&mtx_IOMap);
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        // if (elmo_is_error())
        // {
        //     // pthread_mutex_unlock(&mtx_IOMap);
        //     elmo_set_state(EC_STATE_PRE_OP);
        //     ec_close();
        //     exit(EXIT_FAILURE);
        // }
        pthread_mutex_unlock(&mtx_IOMap);

        next_time.tv_sec += (next_time.tv_nsec + cycletime) / 1e9;
        next_time.tv_nsec = (int)(next_time.tv_nsec + cycletime) % (int)1e9;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
}

OSAL_THREAD_FUNC ecatCheck()
{
    while (inOP)
    {
        if ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)
        {
          /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                      /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}