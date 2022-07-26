#ifndef __ECATERRORHANDLE_HPP__
#define __ECATERRORHANDLE_HPP__

#include <stdio.h>
#include "ethercat.h"

OSAL_THREAD_FUNC_RT sync_thread();
OSAL_THREAD_FUNC ecatCheck();

#endif  // __ECATERRORHANDLE_HPP__
