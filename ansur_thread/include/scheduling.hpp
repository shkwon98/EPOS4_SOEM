#ifndef _GNU_SOURCE1
#define _GNU_SOURCE1

#include <sys/syscall.h>
#include <sys/mman.h>
#include <linux/sched.h>
#include <sys/types.h>
#include <linux/kernel.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>


/* __NR_sched_setattr number */
#ifndef __NR_sched_setattr
#ifdef __x86_64__
#define __NR_sched_setattr      314
#endif

#ifdef __i386__
#define __NR_sched_setattr      351
#endif

#ifdef __arm__
#define __NR_sched_setattr      380
#endif

#ifdef __aarch64__
#define __NR_sched_setattr      274
#endif
#endif

/* __NR_sched_getattr number */
#ifndef __NR_sched_getattr
#ifdef __x86_64__
#define __NR_sched_getattr      315
#endif

#ifdef __i386__
#define __NR_sched_getattr      352
#endif

#ifdef __arm__
#define __NR_sched_getattr      381
#endif

#ifdef __aarch64__CPU_COUNT()
#define __NR_sched_getattr      275
#endif
#endif

#define SCHED_FLAG_DL_OVERRUN		0x04

struct sched_attr
{
    __u32 size;

    __u32 sched_policy;
    __u64 sched_flags;

    /* SCHED_OTHER, SCHED_BATCH */
    __s32 sched_nice;

    /* SCHED_FIFO, SCHED_RR */
    __u32 sched_priority;

    /* SCHED_DEADLINE */
    __u64 sched_runtime;
    __u64 sched_deadline;
    __u64 sched_period;
};

/* use pid=0 to refer to the calling thread */
int sched_setattr(pid_t pid, const struct sched_attr *attr, unsigned int flags);
int sched_getattr(pid_t pid, struct sched_attr *attr, unsigned int size, unsigned int flags);
void sched_deadline(struct sched_attr *attr, long unsigned int runtime, long unsigned int period, long unsigned int deadline, long unsigned int flags);
void sched_rr(struct sched_attr *attr, unsigned int priority, long unsigned int flags);
void sched_fifo(struct sched_attr *attr, unsigned int priority, long unsigned int flags);
void sched_normal(struct sched_attr *attr, int nice, long unsigned int flags);

#endif
