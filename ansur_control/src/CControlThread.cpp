#include "CControlThread.hpp"

extern SERVO_IO_pt EPOS4[NUMOF_EPOS4];
extern bool bRunStart;
extern short mode;
extern INPUT_LIST input;


CControlThread::CControlThread() : CLoopingThread()
{
    udpPacket = new CUdpPacket;
}

CControlThread::~CControlThread() {}

void CControlThread::ec_sync(int64 refTime, int64 cycleTime, int64 *offsetTime)
{
    static int64 integral = 0;
    int64 delta;

    delta = (refTime) % cycleTime;
    if (delta > (cycleTime / 2)) { delta = delta - cycleTime; }
    if (delta > 0) { integral++; }
    if (delta < 0) { integral--; }
    *offsetTime = -(delta / 100) - (integral / 20);
    // g_delta = delta;
}

double CControlThread::sin_motion(double pos_init, double pos_fin, double time_init, double time_fin, double time_now)
{
    double a = pos_init;
    double b = (pos_fin - pos_init);
    double c = (M_PI / ((time_fin - time_init) * 2.0));
    double d = (-1.0) * (time_init);

    return (a + b * sin(c * (time_now + d)));
}

int CControlThread::servo_enable(uint16 StatusWord, uint16 *ControlWord)
{
    int  _enable = 0;
    if (bit_is_clear(StatusWord, STATUSWORD_OPERATION_ENABLE_BIT)) //Not ENABLED yet
    {
        if (bit_is_clear(StatusWord, STATUSWORD_SWITCHED_ON_BIT)) //Not SWITCHED ON yet
        {
            if (bit_is_clear(StatusWord, STATUSWORD_READY_TO_SWITCH_ON_BIT)) //Not READY to SWITCH ON yet
            {
                if (bit_is_set(StatusWord, STATUSWORD_FAULT_BIT)) //FAULT exist
                {
                    (*ControlWord) = 0x80;	//FAULT RESET command
                }
                else //NO FAULT
                {
                    (*ControlWord) = 0x06;	//SHUTDOWN command (transition#2)
                }
            }
            else //READY to SWITCH ON
            {
                (*ControlWord) = 0x07;	//SWITCH ON command (transition#3)
            }
        }
        else //has been SWITCHED ON
        {
            (*ControlWord) = 0x0F;	//ENABLE OPERATION command (transition#4)
            _enable = 1;
        }
    }
    else //has been ENABLED
    {
        (*ControlWord) = 0x0F;	//maintain OPERATION state
        _enable = 1;
    }
    return _enable;;
}

bool CControlThread::task()
{
    uint servo_ready = 0;
    for (int i = 0; i < NUMOF_EPOS4; ++i)
    {
        uint16 controlword = 0;
        started[i] = servo_enable(EPOS4[i].readParam->StatusWord, &controlword);
        servo_ready += started[i];

        EPOS4[i].writeParam->ControlWord = controlword;
    }

    /* BEGIN USER CODE */
    if (servo_ready == NUMOF_EPOS4) // The given task begins here
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
                    EPOS4[0].writeParam->ModeOfOperation = OP_MODE_CYCLIC_SYNC_POSITION;
                    EPOS4[0].writeParam->TargetPosition = (int)CONV_MM_to_INC(tarPos);
                    break;

                case COMMAND_RUN_CSV:  // Velocity Control
                    EPOS4[0].writeParam->ModeOfOperation = OP_MODE_CYCLIC_SYNC_VELOCITY;
                    EPOS4[0].writeParam->TargetVelocity = input.velocity;
                    break;

                case COMMAND_RUN_CST:  // Torque Control
                    EPOS4[0].writeParam->ModeOfOperation = OP_MODE_CYCLIC_SYNC_TORQUE;
                    EPOS4[0].writeParam->TargetTorque = input.torque;
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
                curPos = CONV_INC_to_MM(EPOS4[0].readParam->PositionActualValue);
                bEndFlag = false;
                endCnt = 0;
            }

            tarPos = sin_motion(curPos, 0.0, 0.0, 2000.0, endCnt++);
            if (endCnt >= 2000)
            {
                tarPos = 0;
            }

            EPOS4[0].writeParam->ModeOfOperation = OP_MODE_CYCLIC_SYNC_POSITION;
            EPOS4[0].writeParam->TargetPosition = (int)CONV_MM_to_INC(tarPos);
        }
    }
    // std::cout << "[Target Position: " << EPOS4[0].writeParam->TargetPosition << "] ";
    // std::cout << "| [Actual Position : " << EPOS4[0].readParam->PositionActualValue << "] " << std::endl;

    short header = STREAM_MODE;
    LOG_DATA logData;

    logData.actualVelocity = EPOS4[0].readParam->VelocityActualValue;
    logData.actualTorque = EPOS4[0].readParam->TorqueActualValue;
    logData.actualPosition = EPOS4[0].readParam->PositionActualValue;

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