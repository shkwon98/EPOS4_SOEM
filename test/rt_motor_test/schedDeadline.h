#ifndef SCHEDDEADLINE_H
#define SCHEDDEADLINE_H

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <unistd.h>
#include <stdint.h>
#include <linux/kernel.h>
#include <sys/syscall.h>
#include <sched.h>

struct sched_attr
{
    uint32_t size;
    uint32_t sched_policy;
    uint64_t sched_flags;

    /* used by: SCHED_NORMAL, SCHED_BATCH */
    int32_t sched_nice;

    /* used by: SCHED_FIFO, SCHED_RR */
    uint32_t sched_priority;

    /* used by: SCHED_DEADLINE (nsec) */
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
};

/* sched_policy */
#define SCHED_DEADLINE 6
// #define SYS_sched_yield sched_yield()

/* helpers */
#define gettid() syscall(__NR_gettid)
int sched_setattr(pid_t pid, const struct sched_attr *attr, unsigned int flags)
{
    return syscall(__NR_sched_setattr, pid, attr, flags);
}
int sched_getattr(pid_t pid, struct sched_attr *attr, unsigned int size, unsigned int flags)
{
    return syscall(__NR_sched_getattr, pid, attr, size, flags);
}

#endif /* SCHEDDEADLINE_H */
