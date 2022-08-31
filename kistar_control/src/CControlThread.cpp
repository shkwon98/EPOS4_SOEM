#include "CControlThread.hpp"
#include "csvWrite.hpp"

extern PDO_STRUCT KISTAR[KISTAR_NUM];
extern short mode;
extern JOINT_INPUT_LIST joint_input;
extern MOTION_INPUT_LIST motion_input_1, motion_input_2, motion_input_3, motion_input_4;
extern mutex mtx;

CControlThread::CControlThread() : CLoopingThread()
{
    udpPacket = new CUdpPacket;
}
CControlThread::~CControlThread() { delete udpPacket; }

void CControlThread::printStatus()
{
    std::cout
        << "[Actual OUTPUT] "
        // << "J00: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[0] << " / "
        // << "J01: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[1] << " / "
        // << "J02: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[2] << " / "
        // << "J03: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[3] << " | "

        // << "J04: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[4] << " / "
        << "J05: " << KISTAR[0].read->Actuator_Status1.J5_Servo_On << ", " << std::setw(4) << KISTAR[0].read->JOINT_DATA[5] << " / "
        << "J06: " << KISTAR[0].read->Actuator_Status1.J6_Servo_On << ", " << std::setw(4) << KISTAR[0].read->JOINT_DATA[6] << " / "
        << "J07: " << KISTAR[0].read->Actuator_Status1.J7_Servo_On << ", " << std::setw(4) << KISTAR[0].read->JOINT_DATA[7] << " | "

        // << "J08: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[8] << " / "
        // << "J09: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[9] << " / "
        // << "J10: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[10] << " / "
        // << "J11: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[11] << " | "

        // << "J12: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[12] << " / "
        // << "J13: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[13] << " / "
        // << "J14: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[14] << " / "
        // << "J15: " << std::setw(4) << KISTAR[0].read->JOINT_DATA[15]
        << "  |  \r";
    // fflush(stdout);
}

void CControlThread::sendStatusToGUI()
{
    short header = STREAM_MODE;
    LOG_DATA logData;

    logData.J1_DATA1 = KISTAR[0].read->JOINT_DATA[0];
    logData.J1_DATA2 = KISTAR[0].read->JOINT_DATA[1];
    logData.J1_DATA3 = KISTAR[0].read->JOINT_DATA[2];
    logData.J1_DATA4 = KISTAR[0].read->JOINT_DATA[3];

    logData.J2_DATA1 = KISTAR[0].read->JOINT_DATA[4];
    logData.J2_DATA2 = KISTAR[0].read->JOINT_DATA[5];
    logData.J2_DATA3 = KISTAR[0].read->JOINT_DATA[6];
    logData.J2_DATA4 = KISTAR[0].read->JOINT_DATA[7];

    logData.J3_DATA1 = KISTAR[0].read->JOINT_DATA[8];
    logData.J3_DATA2 = KISTAR[0].read->JOINT_DATA[9];
    logData.J3_DATA3 = KISTAR[0].read->JOINT_DATA[10];
    logData.J3_DATA4 = KISTAR[0].read->JOINT_DATA[11];

    logData.J4_DATA1 = KISTAR[0].read->JOINT_DATA[12];
    logData.J4_DATA2 = KISTAR[0].read->JOINT_DATA[13];
    logData.J4_DATA3 = KISTAR[0].read->JOINT_DATA[14];
    logData.J4_DATA4 = KISTAR[0].read->JOINT_DATA[15];

    udpPacket->setCommandHeader(header);
    udpPacket->encode(logData);
    udpPacket->sendPacket();
}

double sinWave(double amplitude, double period, double offset, unsigned int tick)
{
    return (offset + amplitude * sin((2 * M_PI / period) * CONTROL_PERIOD_IN_s * tick));
}
void CControlThread::controlWithGUI()
{
    switch (mode)
    {
        case MODE_OFF:
            break;

        case MODE_JOINT_POSITION:
            // KISTAR[0].write->JOINT_TARGET[5] = joint_input.J2_TARGET2;
            // KISTAR[0].write->JOINT_TARGET[6] = joint_input.J2_TARGET3;
            // KISTAR[0].write->JOINT_TARGET[7] = joint_input.J2_TARGET4;
            break;

        case MODE_MOTION:
            // KISTAR[0].write->JOINT_TARGET[5] = (int16)sinWave(motion_input_1.amplitude, motion_input_1.period, 0, tick);
            // KISTAR[0].write->JOINT_TARGET[6] = (int16)sinWave(motion_input_1.amplitude, motion_input_1.period, 0, tick);
            // KISTAR[0].write->JOINT_TARGET[7] = (int16)sinWave(motion_input_1.amplitude, motion_input_1.period, 0, tick);
            // tick++;
            break;

        default:
            break;
    }

    sendStatusToGUI();
}
void CControlThread::controlStandAlone()
{
    if (tick == 2000)
    {
        KISTAR[0].write->Actuator_Status1.J5_Servo_On = 0;
        KISTAR[0].write->Actuator_Status1.J6_Servo_On = 0;
        KISTAR[0].write->Actuator_Status1.J7_Servo_On = 0;
    }
    // KISTAR[0].write->JOINT_TARGET[5] = (int16)(-100);
    // KISTAR[0].write->JOINT_TARGET[6] = (int16)(-100);
    // KISTAR[0].write->JOINT_TARGET[7] = (int16)(-100);
    tick++;
}


/** Redefinition of pure virtual method in parent class. The real-time task of ethercat communication.
 *
*/
void CControlThread::task()
{
    if (SOEM::inOP)
    {
#ifdef USE_GUI
        controlWithGUI();
#else
        controlStandAlone();
#endif

        printStatus();
        // mtx.lock();
        ec_send_processdata();
        SOEM::wkc = ec_receive_processdata(EC_TIMEOUTRET);
        // mtx.unlock();
    }

    if (ec_slave[0].hasdc)  // calculate toff to get linux time and DC synced
    {
        ec_sync(ec_DCtime, m_period, &toff);
        m_addtime = m_period + toff;
    }
}




////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////


// void CControlThread::motorTorqueOn()
// {
//     isMotorTorqueOn = false;
//     uint servo_ready = 0;
//     int started[EPOS4_NUM] = { 0 };
//
//     for (int i = 0; i < EPOS4_NUM; ++i)
//     {
//         uint16 controlword = 0;
//         started[i] = servo_enable(EPOS4[i].read->StatusWord, &controlword);
//         servo_ready += started[i];
//
//         EPOS4[i].write->ControlWord = controlword;
//     }
//     if (servo_ready == EPOS4_NUM)
//     {
//         isMotorTorqueOn = true;
//     }
// }
// void CControlThread::motorTorqueOff()
// {
//     for (int i = 0; i < EPOS4_NUM; ++i)
//     {
//         EPOS4[i].write->ControlWord = CONTROL_COMMAND_DIABLE_VOLTAGE;
//     }
//     mtx.lock();
//     ec_send_processdata();
//     SOEM::wkc = ec_receive_processdata(EC_TIMEOUTRET);
//     mtx.unlock();
// }

// double CControlThread::sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now)
// {
//     double a = pos_init;
//     double b = (pos_fin - pos_init);
//     double c = (M_PI / ((time_fin - time_init) * 2.0));
//     double d = (-1.0) * (time_init);
//     return (a + b * sin(c * (time_now + d)));
// }