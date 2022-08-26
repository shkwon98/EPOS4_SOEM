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

    // vec1.reserve(70000);
    // vec2.reserve(70000);
    // vec3.reserve(70000);
    // vec4.reserve(70000);
    // vec5.reserve(70000);
    // vec6.reserve(70000);
    // vec7.reserve(70000);
    // vec8.reserve(70000);
    // vec9.reserve(70000);
}
CControlThread::~CControlThread() { delete udpPacket; }

void CControlThread::printStatus()
{
    // J1
    // std::cout << "[TARGET INPUT] "
    //     << "J11: " << std::setw(4) << KISTAR[0].write->J1_TARGET1 << ", "
    //     << "J12: " << std::setw(4) << KISTAR[0].write->J1_TARGET2 << ", "
    //     << "J13: " << std::setw(4) << KISTAR[0].write->J1_TARGET3 << ", "
    //     << "J14: " << std::setw(4) << KISTAR[0].write->J1_TARGET4 << " | "
    //     << "[Actual OUTPUT] "
    //     << "J11: " << std::setw(4) << KISTAR[0].read->J1_DATA1 << ", "
    //     << "J12: " << std::setw(4) << KISTAR[0].read->J1_DATA2 << ", "
    //     << "J13: " << std::setw(4) << KISTAR[0].read->J1_DATA3 << ", "
    //     << "J14: " << std::setw(4) << KISTAR[0].read->J1_DATA4
    //     << "  ||  \r";

    // J2
    // std::cout << "[TARGET] "
    //     << "J21: " << std::setw(4) << KISTAR[0].write->J2_TARGET1 << ", "
    //     << "J22: " << std::setw(4) << KISTAR[0].write->J2_TARGET2 << ", "
    //     << "J23: " << std::setw(4) << KISTAR[0].write->J2_TARGET3 << ", "
    //     << "J24: " << std::setw(4) << KISTAR[0].write->J2_TARGET4 << " | "
    //     << "[Actual] "
    //     << "J21: " << std::setw(4) << KISTAR[0].read->J2_DATA1 << ", "
    //     << "J22: " << std::setw(4) << KISTAR[0].read->J2_DATA2 << ", "
    //     << "J23: " << std::setw(4) << KISTAR[0].read->J2_DATA3 << ", "
    //     << "J24: " << std::setw(4) << KISTAR[0].read->J2_DATA4
    //     << "  ||  \r";

    std::cout
        << "[Actual OUTPUT] "
        << "J11: " << std::setw(4) << KISTAR[0].read->J1_DATA1 << ", "
        << "J12: " << std::setw(4) << KISTAR[0].read->J1_DATA2 << ", "
        << "J13: " << std::setw(4) << KISTAR[0].read->J1_DATA3 << ", "
        << "J14: " << std::setw(4) << KISTAR[0].read->J1_DATA4 << " | "

        << "J21: " << std::setw(4) << KISTAR[0].read->J2_DATA1 << ", "
        << "J22: " << std::setw(4) << KISTAR[0].read->J2_DATA2 << ", "
        << "J23: " << std::setw(4) << KISTAR[0].read->J2_DATA3 << ", "
        << "J24: " << std::setw(4) << KISTAR[0].read->J2_DATA4 << " | "

        << "J31: " << std::setw(4) << KISTAR[0].read->J3_DATA1 << ", "
        << "J32: " << std::setw(4) << KISTAR[0].read->J3_DATA2 << ", "
        << "J33: " << std::setw(4) << KISTAR[0].read->J3_DATA3 << ", "
        << "J34: " << std::setw(4) << KISTAR[0].read->J3_DATA4 << " | "

        << "J41: " << std::setw(4) << KISTAR[0].read->J4_DATA1 << ", "
        << "J42: " << std::setw(4) << KISTAR[0].read->J4_DATA2 << ", "
        << "J43: " << std::setw(4) << KISTAR[0].read->J4_DATA3 << ", "
        << "J44: " << std::setw(4) << KISTAR[0].read->J4_DATA4
        << "  ||  \r";

    fflush(stdout);
}

void CControlThread::sendStatusToGui()
{
    short header = STREAM_MODE;
    LOG_DATA logData;

    logData.J1_DATA1 = KISTAR[0].read->J1_DATA1;
    logData.J1_DATA2 = KISTAR[0].read->J1_DATA2;
    logData.J1_DATA3 = KISTAR[0].read->J1_DATA3;
    logData.J1_DATA4 = KISTAR[0].read->J1_DATA4;

    logData.J2_DATA1 = KISTAR[0].read->J2_DATA1;
    logData.J2_DATA2 = KISTAR[0].read->J2_DATA2;
    logData.J2_DATA3 = KISTAR[0].read->J2_DATA3;
    logData.J2_DATA4 = KISTAR[0].read->J2_DATA4;

    logData.J3_DATA1 = KISTAR[0].read->J3_DATA1;
    logData.J3_DATA2 = KISTAR[0].read->J3_DATA2;
    logData.J3_DATA3 = KISTAR[0].read->J3_DATA3;
    logData.J3_DATA4 = KISTAR[0].read->J3_DATA4;

    logData.J4_DATA1 = KISTAR[0].read->J4_DATA1;
    logData.J4_DATA2 = KISTAR[0].read->J4_DATA2;
    logData.J4_DATA3 = KISTAR[0].read->J4_DATA3;
    logData.J4_DATA4 = KISTAR[0].read->J4_DATA4;

    udpPacket->setCommandHeader(header);
    udpPacket->encode(logData);
    udpPacket->sendPacket();
}

double CControlThread::sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now)
{
    double a = pos_init;
    double b = (pos_fin - pos_init);
    double c = (M_PI / ((time_fin - time_init) * 2.0));
    double d = (-1.0) * (time_init);

    return (a + b * sin(c * (time_now + d)));
}

double CControlThread::sinWave(double amplitude, double period, double offset)
{
    return (offset + amplitude * sin((2 * M_PI / period) * CONTROL_PERIOD_IN_s * tick));
}

void CControlThread::controlWithGUI()
{
    switch (mode)
    {
        case MODE_OFF:
            // tick = 0;
            // // Initialization
            // if (bEndFlag)
            // {
            //     curPos = CONV_INC_to_MM(EPOS4[0].read->PositionActualValue);
            //     bEndFlag = false;
            //     endCnt = 0;
            // }
            // tarPos = sin_motion(curPos, 0.0, 0.0, 2000.0, endCnt++);
            // if (endCnt >= 2000)
            // {
            //     tarPos = 0;
            // }
            // EPOS4[0].write->ModeOfOperation = OP_MODE_CYCLIC_SYNC_POSITION;
            // EPOS4[0].write->TargetPosition = (int)CONV_MM_to_INC(tarPos);
            break;

        case MODE_JOINT_POSITION:
            KISTAR[0].write->J1_TARGET1 = joint_input.J1_TARGET1;
            KISTAR[0].write->J1_TARGET2 = joint_input.J1_TARGET2;
            KISTAR[0].write->J1_TARGET3 = joint_input.J1_TARGET3;
            KISTAR[0].write->J1_TARGET4 = joint_input.J1_TARGET4;

            KISTAR[0].write->J2_TARGET1 = joint_input.J2_TARGET1;
            KISTAR[0].write->J2_TARGET2 = joint_input.J2_TARGET2;
            KISTAR[0].write->J2_TARGET3 = joint_input.J2_TARGET3;
            KISTAR[0].write->J2_TARGET4 = joint_input.J2_TARGET4;

            KISTAR[0].write->J3_TARGET1 = joint_input.J3_TARGET1;
            KISTAR[0].write->J3_TARGET2 = joint_input.J3_TARGET2;
            KISTAR[0].write->J3_TARGET3 = joint_input.J3_TARGET3;
            KISTAR[0].write->J3_TARGET4 = joint_input.J3_TARGET4;

            KISTAR[0].write->J4_TARGET1 = joint_input.J4_TARGET1;
            KISTAR[0].write->J4_TARGET2 = joint_input.J4_TARGET2;
            KISTAR[0].write->J4_TARGET3 = joint_input.J4_TARGET3;
            KISTAR[0].write->J4_TARGET4 = joint_input.J4_TARGET4;
            break;

        case MODE_MOTION:
            KISTAR[0].write->J1_TARGET1 = (int16)sinWave(motion_input_1.amplitude, motion_input_1.period, 0);
            KISTAR[0].write->J1_TARGET2 = (int16)sinWave(motion_input_1.amplitude, motion_input_1.period, 0);
            KISTAR[0].write->J1_TARGET3 = (int16)sinWave(motion_input_1.amplitude, motion_input_1.period, 0);
            KISTAR[0].write->J1_TARGET4 = (int16)sinWave(motion_input_1.amplitude, motion_input_1.period, 0);

            KISTAR[0].write->J2_TARGET1 = (int16)sinWave(motion_input_2.amplitude, motion_input_2.period, 0);
            KISTAR[0].write->J2_TARGET2 = (int16)sinWave(motion_input_2.amplitude, motion_input_2.period, 0);
            KISTAR[0].write->J2_TARGET3 = (int16)sinWave(motion_input_2.amplitude, motion_input_2.period, 0);
            KISTAR[0].write->J2_TARGET4 = (int16)sinWave(motion_input_2.amplitude, motion_input_2.period, 0);

            KISTAR[0].write->J3_TARGET1 = (int16)sinWave(motion_input_3.amplitude, motion_input_3.period, 0);
            KISTAR[0].write->J3_TARGET2 = (int16)sinWave(motion_input_3.amplitude, motion_input_3.period, 0);
            KISTAR[0].write->J3_TARGET3 = (int16)sinWave(motion_input_3.amplitude, motion_input_3.period, 0);
            KISTAR[0].write->J3_TARGET4 = (int16)sinWave(motion_input_3.amplitude, motion_input_3.period, 0);

            KISTAR[0].write->J4_TARGET1 = (int16)sinWave(motion_input_4.amplitude, motion_input_4.period, 0);
            KISTAR[0].write->J4_TARGET2 = (int16)sinWave(motion_input_4.amplitude, motion_input_4.period, 0);
            KISTAR[0].write->J4_TARGET3 = (int16)sinWave(motion_input_4.amplitude, motion_input_4.period, 0);
            KISTAR[0].write->J4_TARGET4 = (int16)sinWave(motion_input_4.amplitude, motion_input_4.period, 0);

            tick++;
            break;

        default:
            break;
    }

    sendStatusToGui();
}
void CControlThread::controlTest()
{
    KISTAR[0].write->J1_TARGET1 = (int16)sinWave(1000, 0.25, 0);
    KISTAR[0].write->J1_TARGET2 = (int16)sinWave(1000, 0.5, 0);
    KISTAR[0].write->J1_TARGET3 = (int16)sinWave(1000, 1, 0);
    KISTAR[0].write->J1_TARGET4 = (int16)sinWave(1000, 2, 0);

    vec1.push_back((int)tick);
    vec2.push_back((int)(KISTAR[0].write->J1_TARGET1));
    vec3.push_back((int)(KISTAR[0].write->J1_TARGET2));
    vec4.push_back((int)(KISTAR[0].write->J1_TARGET3));
    vec5.push_back((int)(KISTAR[0].write->J1_TARGET4));
    vec6.push_back((int)(KISTAR[0].read->J1_DATA1));
    vec7.push_back((int)(KISTAR[0].read->J1_DATA2));
    vec8.push_back((int)(KISTAR[0].read->J1_DATA3));
    vec9.push_back((int)(KISTAR[0].read->J1_DATA4));

    tick++;
}

void CControlThread::task()
{
    if (SOEM::inOP)
    {
        /******  MOTOR CONTROL COMMAND  ******/
        controlWithGUI();
        // controlTest();

        /*******  PRINT MOTOR STATUS  *******/
        printStatus();


        // mtx.lock();
        ec_send_processdata();
        SOEM::wkc = ec_receive_processdata(EC_TIMEOUTRET);
        // mtx.unlock();
    }
    // else
    // {
    //     std::vector<std::vector<int>> vals = { vec1, vec2, vec3, vec4, vec5, vec6, vec7, vec8, vec9 };
    //     write_csv("/home/pi/workspace/1.KISTAR_Hand_SOEM/log/KISTAR_IO_log.csv", vals);
    // }

    if (ec_slave[0].hasdc)  // calculate toff to get linux time and DC synced
    {
        SOEM::ec_sync(ec_DCtime, m_period, &toff);
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