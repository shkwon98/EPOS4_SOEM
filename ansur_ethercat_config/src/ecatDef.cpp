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

