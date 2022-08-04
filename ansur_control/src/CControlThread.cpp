#include "CControlThread.hpp"

extern PDO_STRUCT EPOS4[EPOS4_NUM];
extern bool bRunStart;
extern short mode;
extern INPUT_LIST input;

CControlThread::CControlThread() : CLoopingThread() { udpPacket = new CUdpPacket; }
CControlThread::~CControlThread() { delete udpPacket; }

void CControlThread::motorTorqueOn()
{
    isMotorTorqueOn = false;

    uint servo_ready = 0;
    started[EPOS4_NUM] = { 0 };
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
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
}

double CControlThread::sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now)
{
    double a = pos_init;
    double b = (pos_fin - pos_init);
    double c = (M_PI / ((time_fin - time_init) * 2.0));
    double d = (-1.0) * (time_init);

    return (a + b * sin(c * (time_now + d)));
}

void CControlThread::task()
{
    if (SOEM::inOP)
    {
        motorTorqueOn();

        if (isMotorTorqueOn == true)
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
            // std::cout << "[Target Position: " << EPOS4[0].write->TargetPosition << "] ";
            // std::cout << "| [Actual Position : " << EPOS4[0].read->PositionActualValue << "] " << std::endl;
        }
    }
    else { motorTorqueOff(); }

    sendMotorData();
}

void CControlThread::sendMotorData()
{
    short header = STREAM_MODE;
    LOG_DATA logData;

    logData.actualVelocity = EPOS4[0].read->VelocityActualValue;
    logData.actualTorque = EPOS4[0].read->TorqueActualValue;
    logData.actualPosition = EPOS4[0].read->PositionActualValue;

    udpPacket->setCommandHeader(header);
    udpPacket->encode(logData);
    udpPacket->sendPacket();

    if (ec_slave[0].hasdc)  /* calculate toff to get linux time and DC synced */
    {
        SOEM::ec_sync(ec_DCtime, m_period, &toff);
        m_addtime = m_period + toff;
    }
}