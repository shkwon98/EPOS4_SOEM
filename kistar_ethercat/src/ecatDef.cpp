#include "ecatDef.hpp"

int servo_enable(uint16 StatusWord, uint16 * ControlWord)
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
                    (*ControlWord) = CONTROL_COMMAND_FAULT_RESET;	//FAULT RESET command
                }
                else //NO FAULT
                {
                    (*ControlWord) = CONTROL_COMMAND_SHUTDOWN;	//SHUTDOWN command (transition#2)
                }
            }
            else //READY to SWITCH ON
            {
                (*ControlWord) = CONTROL_COMMAND_SWITCH_ON;	//SWITCH ON command (transition#3)
            }
        }
        else //has been SWITCHED ON
        {
            (*ControlWord) = CONTROL_COMMAND_SWITCH_ON_ENABLE_OPERATION;	//ENABLE OPERATION command (transition#4)
            _enable = 1;
        }
    }
    else //has been ENABLED
    {
        (*ControlWord) = CONTROL_COMMAND_ENABLE_OPERATION;	//maintain OPERATION state
        _enable = 1;
    }
    return _enable;;
}

