#include "CControlThread.hpp"

extern SERVO_IO::SERVO_READ  *EPOS4_READ[EPOS4_NUM];
extern SERVO_IO::SERVO_WRITE *EPOS4_WRITE[EPOS4_NUM];

extern bool bRunStart;
extern short mode;
extern INPUT_LIST input;


CControlThread::CControlThread() : CLoopingThread()
{
    udpPacket = new CUdpPacket;
}

CControlThread::~CControlThread() {}


double CControlThread::sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now)
{
    double a = pos_init;
    double b = (pos_fin - pos_init);
    double c = (M_PI / ((time_fin - time_init) * 2.0));
    double d = (-1.0) * (time_init);

    return (a + b * sin(c * (time_now + d)));
}

bool CControlThread::task()
{
    uint servo_ready = 0;
    for (int i = 0; i < EPOS4_NUM; ++i)
    {
        uint16 controlword = 0;
        started[i] = servo_enable(EPOS4_READ[i]->StatusWord, &controlword);
        servo_ready += started[i];

        EPOS4_WRITE[i]->ControlWord = controlword;
    }

    /* BEGIN USER CODE */
    if (servo_ready == EPOS4_NUM) // The given task begins here
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
                    EPOS4_WRITE[0]->ModeOfOperation = OP_MODE_CYCLIC_SYNC_POSITION;
                    EPOS4_WRITE[0]->TargetPosition = (int)CONV_MM_to_INC(tarPos);
                    break;

                case COMMAND_RUN_CSV:  // Velocity Control
                    EPOS4_WRITE[0]->ModeOfOperation = OP_MODE_CYCLIC_SYNC_VELOCITY;
                    EPOS4_WRITE[0]->TargetVelocity = input.velocity;
                    break;

                case COMMAND_RUN_CST:  // Torque Control
                    EPOS4_WRITE[0]->ModeOfOperation = OP_MODE_CYCLIC_SYNC_TORQUE;
                    EPOS4_WRITE[0]->TargetTorque = input.torque;
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
                curPos = CONV_INC_to_MM(EPOS4_READ[0]->PositionActualValue);
                bEndFlag = false;
                endCnt = 0;
            }

            tarPos = sin_motion(curPos, 0.0, 0.0, 2000.0, endCnt++);
            if (endCnt >= 2000)
            {
                tarPos = 0;
            }

            EPOS4_WRITE[0]->ModeOfOperation = OP_MODE_CYCLIC_SYNC_POSITION;
            EPOS4_WRITE[0]->TargetPosition = (int)CONV_MM_to_INC(tarPos);
        }
    }
    // std::cout << "[Target Position: " << EPOS4_WRITE[0]->TargetPosition << "] ";
    // std::cout << "| [Actual Position : " << EPOS4_READ[0]->PositionActualValue << "] " << std::endl;

    short header = STREAM_MODE;
    LOG_DATA logData;

    logData.actualVelocity = EPOS4_READ[0]->VelocityActualValue;
    logData.actualTorque = EPOS4_READ[0]->TorqueActualValue;
    logData.actualPosition = EPOS4_READ[0]->PositionActualValue;

    udpPacket->setCommandHeader(header);
    udpPacket->encode(logData);
    udpPacket->sendPacket();

    if (ec_slave[0].hasdc)  /* calculate toff to get linux time and DC synced */
    {
        ec_sync(ec_DCtime, m_period, &toff);
        m_addtime = m_period + toff;
    }

    return 0;
}