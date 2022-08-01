#ifndef TCPCOMMAND_HPP
#define TCPCOMMAND_HPP

#include "CTcpPacket.hpp"
#include "ethercat.h"
#include "SOEM.hpp"

OSAL_THREAD_FUNC tcpCommand(void);

#endif // TCPCOMMAND_HPP
