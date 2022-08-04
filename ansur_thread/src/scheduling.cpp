#include "scheduling.hpp"


/* use pid=0 for the calling thread */
int sched_setattr(pid_t pid, const struct sched_attr *attr, unsigned int flags)
{
    return syscall(__NR_sched_setattr, pid, attr, flags);
}

int sched_getattr(pid_t pid, struct sched_attr *attr, unsigned int size, unsigned int flags)
{
    return syscall(__NR_sched_getattr, pid, attr, size, flags);
}

void sched_deadline(struct sched_attr *attr, long unsigned int runtime, long unsigned int period, long unsigned int deadline, long unsigned int flags)
{

    int ret;

    attr->sched_nice = 0;
    attr->sched_priority = 0;
    attr->sched_policy = SCHED_DEADLINE;
    attr->sched_runtime = runtime;
    attr->sched_period = period;
    attr->sched_deadline = deadline;
    attr->sched_flags = flags;

    ret = sched_setattr(0, attr, 0);

    if (ret < 0)
    {
        perror("sched_setattr failed to set ");
        exit(-1);
    }
}

void sched_fifo(struct sched_attr * attr, unsigned int priority, long unsigned int flags)
{

    int ret;

    attr->sched_nice = 0;
    attr->sched_policy = SCHED_FIFO;
    attr->sched_priority = priority;
    attr->sched_flags = flags;

    ret = sched_setattr(0, attr, 0);

    if (ret < 0)
    {
        perror("sched_setattr failed to set the priorities");
        exit(-1);
    }
}

void sched_rr(struct sched_attr *attr, unsigned int priority, long unsigned int flags)
{


    int ret;

    attr->sched_nice = 0;
    attr->sched_policy = SCHED_RR;
    attr->sched_priority = priority;
    attr->sched_flags = flags;

    ret = sched_setattr(0, attr, 0);

    if (ret < 0)
    {
        perror("sched_setattr failed to set the priorities");
        exit(-1);
    }
}

void sched_normal(struct sched_attr *attr, int nice, long unsigned int flags)
{


    int ret;

    attr->sched_policy = SCHED_NORMAL;
    attr->sched_priority = 0;
    attr->sched_nice = nice;
    attr->sched_flags = flags;

    ret = sched_setattr(0, attr, 0);

    if (ret < 0)
    {
        perror("sched_setattr failed to set the priorities");
        exit(-1);
    }
}


