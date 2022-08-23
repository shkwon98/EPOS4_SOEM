#include "CControlThread.hpp"
#include "csvWrite.hpp"

extern PDO_STRUCT EPOS4[EPOS4_NUM];
extern bool bRunStart;
extern short mode;
extern INPUT_LIST input;
extern mutex mtx;

CControlThread::CControlThread() : CLoopingThread()
{
    udpPacket = new CUdpPacket;

    // vector<int> vec1;
    // vector<int> vec2;
    // vector<int> vec3;
    // vec1.reserve(70000);
    // vec2.reserve(70000);
    // vec3.reserve(70000);
}
CControlThread::~CControlThread() { delete udpPacket; }

void CControlThread::motorTorqueOn()
{
    isMotorTorqueOn = false;
    uint servo_ready = 0;
    int started[EPOS4_NUM] = { 0 };

    for (int i = 0; i < EPOS4_NUM; ++i)
    {
        uint16 controlword = 0;
        started[i] = servo_enable(EPOS4[i].read->StatusWord, &controlword);
        servo_ready += started[i];

        EPOS4[i].write->ControlWord = controlword;
    }
    if (servo_ready == EPOS4_NUM)
    {
        isMotorTorqueOn = true;
    }
}
void CControlThread::motorTorqueOff()
{
    for (int i = 0; i < EPOS4_NUM; ++i)
    {
        EPOS4[i].write->ControlWord = CONTROL_COMMAND_DIABLE_VOLTAGE;
    }
    mtx.lock();
    ec_send_processdata();
    SOEM::wkc = ec_receive_processdata(EC_TIMEOUTRET);
    mtx.unlock();
}

void CControlThread::sendMotorState()
{
    short header = STREAM_MODE;
    LOG_DATA logData;

    logData.actualVelocity = EPOS4[0].read->VelocityActualValue;
    logData.actualTorque = EPOS4[0].read->TorqueActualValue;
    logData.actualPosition = EPOS4[0].read->PositionActualValue;

    udpPacket->setCommandHeader(header);
    udpPacket->encode(logData);
    udpPacket->sendPacket();
}
void CControlThread::printMotorStatus()
{
    // MOTOR 1
    std::cout << "[MOTOR1] "
        << "Target Pos: " << std::setw(4) << EPOS4[0].write->TargetPosition << ", "
        << "Target Toq: " << std::setw(4) << EPOS4[0].write->TargetTorque << ", "
        << "Target Vel: " << std::setw(4) << EPOS4[0].write->TargetVelocity << " | "
        << "Actual Pos: " << std::setw(4) << EPOS4[0].read->PositionActualValue << ", "
        << "Actual Toq: " << std::setw(4) << EPOS4[0].read->TorqueActualValue << ", "
        << "Actual Vel: " << std::setw(4) << EPOS4[0].read->VelocityActualValue
        << "  ||  \r";

    // MOTOR 2
    // std::cout << "[MOTOR2] "
    //     << "Target Pos: " << std::setw(4) << EPOS4[1].write->TargetPosition << ", "
    //     << "Target Toq: " << std::setw(4) << EPOS4[1].write->TargetTorque << ", "
    //     << "Target Vel: " << std::setw(4) << EPOS4[1].write->TargetVelocity << " | "
    //     << "Actual Pos: " << std::setw(4) << EPOS4[1].read->PositionActualValue << ", "
    //     << "Actual Toq: " << std::setw(4) << EPOS4[1].read->TorqueActualValue << ", "
    //     << "Actual Vel: " << std::setw(4) << EPOS4[1].read->VelocityActualValue
    //     << " \r";

    fflush(stdout);
}

double CControlThread::sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now)
{
    double a = pos_init;
    double b = (pos_fin - pos_init);
    double c = (M_PI / ((time_fin - time_init) * 2.0));
    double d = (-1.0) * (time_init);

    return (a + b * sin(c * (time_now + d)));
}

void CControlThread::controlWithGUI()
{
    switch (input.taskParam.taskType)
    {
        case RUN_TASK_NONE:
            tarPos = 0;
            break;

        case RUN_TASK_MOTOR_CONTROL:
        {
            double dTriPeriod = (2 * M_PI) * CONTROL_PERIOD_IN_s / (input.taskParam.period);
            tarPos = input.taskParam.disp * sin(dTriPeriod * tick++);
            break;
        }

        default:
            tarPos = 0;
            break;
    }

    if (bRunStart)  // Control ON
    {
        switch (mode)
        {
            case COMMAND_RUN_CSP:  // Position Control
                EPOS4[0].write->ModeOfOperation = OP_MODE_CYCLIC_SYNC_POSITION;
                EPOS4[0].write->TargetPosition = (int)CONV_MM_to_INC(tarPos);
                break;

            case COMMAND_RUN_CSV:  // Velocity Control
                EPOS4[0].write->ModeOfOperation = OP_MODE_CYCLIC_SYNC_VELOCITY;
                EPOS4[0].write->TargetVelocity = input.velocity;
                break;

            case COMMAND_RUN_CST:  // Torque Control
                EPOS4[0].write->ModeOfOperation = OP_MODE_CYCLIC_SYNC_TORQUE;
                EPOS4[0].write->TargetTorque = input.torque;
                break;
        }
        bEndFlag = true;
    }

    else  // Control OFF
    {
        tick = 0;

        // Initialization
        if (bEndFlag)
        {
            curPos = CONV_INC_to_MM(EPOS4[0].read->PositionActualValue);
            bEndFlag = false;
            endCnt = 0;
        }

        tarPos = sin_motion(curPos, 0.0, 0.0, 2000.0, endCnt++);
        if (endCnt >= 2000)
        {
            tarPos = 0;
        }

        EPOS4[0].write->ModeOfOperation = OP_MODE_CYCLIC_SYNC_POSITION;
        EPOS4[0].write->TargetPosition = (int)CONV_MM_to_INC(tarPos);
    }

    sendMotorState();
}
void CControlThread::controlTest()
{
    EPOS4[0].write->ModeOfOperation = OP_MODE_CYCLIC_SYNC_VELOCITY;
    // EPOS4[1].write->ModeOfOperation = OP_MODE_CYCLIC_SYNC_VELOCITY;
    EPOS4[0].write->TargetVelocity = 10000;
    // EPOS4[1].write->TargetVelocity = 10000;


    // double dTriPeriod = (2 * M_PI) / 5;
    // tarPos = 1 * sin(dTriPeriod * CONTROL_PERIOD_IN_s * tick++);
    // EPOS4[0].write->ModeOfOperation = OP_MODE_CYCLIC_SYNC_POSITION;
    // EPOS4[0].write->TargetPosition = (int)CONV_MM_to_INC(tarPos);
}

void CControlThread::task()
{
    if (SOEM::inOP)
    {
        motorTorqueOn();

        if (isMotorTorqueOn == true)
        {
            /******  MOTOR CONTROL COMMAND  ******/
            controlWithGUI();
            // controlTest();

            /*******  PRINT MOTOR STATUS  *******/
            printMotorStatus();


            // vec1.push_back(EPOS4[0].write->TargetVelocity);
            // vec2.push_back(EPOS4[0].read->VelocityActualValue);
            // vec3.push_back(EPOS4[1].read->VelocityActualValue);
        }

        // mtx.lock();
        ec_send_processdata();
        SOEM::wkc = ec_receive_processdata(EC_TIMEOUTRET);
        // mtx.unlock();
    }
    else
    {
        motorTorqueOff();

        // std::vector<std::vector<int>> vals = { vec1, vec2 };
        // write_csv("/home/pi/workspace/1.Ansur_SOEM/log/1ms_velocity_3.csv", vals);
    }

    if (ec_slave[0].hasdc)  // calculate toff to get linux time and DC synced
    {
        SOEM::ec_sync(ec_DCtime, m_period, &toff);
        m_addtime = m_period + toff;
    }
}