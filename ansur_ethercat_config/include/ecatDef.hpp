#ifndef ECATDEF_HPP
#define ECATDEF_HPP

#include <sys/mman.h>
#include <iostream>
#include "ethercat.h"

#define _BV(bit)                (1 << (bit))
#define bit_is_set(val, bit)    (val & _BV(bit))
#define bit_is_clear(val, bit)  (!(val & _BV(bit)))
#define sbi(val, bit)           ((val) |= _BV(bit))
#define cbi(val, bit)           ((val) &= ~_BV(bit))

#define STATUSWORD_READY_TO_SWITCH_ON_BIT       0
#define STATUSWORD_SWITCHED_ON_BIT              1
#define STATUSWORD_OPERATION_ENABLE_BIT         2
#define STATUSWORD_FAULT_BIT                    3
#define STATUSWORD_VOLTAGE_ENABLE_BIT           4
#define STATUSWORD_QUICK_STOP_BIT               5
#define STATUSWORD_SWITCH_ON_DISABLE_BIT        6
#define STATUSWORD_NO_USED_WARNING_BIT          7
#define STATUSWORD_ELMO_NOT_USED_BIT            8
#define STATUSWORD_REMOTE_BIT                   9
#define STATUSWORD_TARGET_REACHED_BIT           10
#define STATUSWORD_INTERNAL_LIMIT_ACTIVE_BIT    11
#define STATUSWORD_HOMING_ATTAINED_BIT          12

/** Control CoE FSM(Finite State Machine) **/
#define EC_CONTROL_COMMAND                          0x02
#define CONTROL_COMMAND_DIABLE_VOLTAGE              0x0000
#define CONTROL_COMMAND_QUICK_STOP                  0x0002
#define CONTROL_COMMAND_SHUTDOWN                    0x0006
#define CONTROL_COMMAND_SWITCH_ON                   0x0007
#define CONTROL_COMMAND_ENABLE_OPERATION            0x000F
#define CONTROL_COMMAND_SWITCH_ON_ENABLE_OPERATION  0x000F
#define CONTROL_COMMAND_DISABLE_OPERATION           0x0007
#define CONTROL_COMMAND_FAULT_RESET                 0x0080

/** Operation mode **/
#define EC_OP_MODE                          0x03

#define OP_MODE_NO_MODE                     0x00
#define OP_MODE_PROFILE_POSITION            0x01
#define OP_MODE_VELOCITY                    0x02
#define OP_MODE_PROFILE_VELOCITY            0x03
#define OP_MODE_TORQUE_PROFILE              0x04
#define OP_MODE_HOMING                      0x06
#define OP_MODE_INTERPOLATED_POSITION       0x07
#define OP_MODE_CYCLIC_SYNC_POSITION        0x08
#define OP_MODE_CYCLIC_SYNC_VELOCITY        0x09
#define OP_MODE_CYCLIC_SYNC_TORQUE          0x0A

#define SERVO_TIMEOUT                       50000

#ifndef PI
#define PI      3.14159265359
#define RAD2DEG     57.2957795131
#define DEG2RAD     0.01745329252
#endif


int servo_enable(uint16 StatusWord, uint16 * ControlWord);


// Structure representing the Object Dictionary
typedef struct _mapping_obj
{
    int32 Index;
    int32 SubIndex;
    int32 Size;
    int32 Value;
}mapping_obj;

///////////////// PDO Setting /////////////////
#pragma pack(push,1)
struct SERVO_IO
{
    struct SERVO_WRITE  // 0x1600 RxPDO
    {
        uint16  ControlWord;        // 0x6040
        int8    ModeOfOperation;    // 0x6060
        int16   TargetTorque;       // 0x6071
        int32   TargetVelocity;     // 0x60FF
        int32   TargetPosition;     // 0x607A
        uint32  ProfileVelocity;    // 0x6081
        // int32  PositionOffset;     // 0x60B0
        // int32  VelocityOffset;     // 0x60B1
        // int16  TorqueOffset;       // 0x60B2
    };

    struct SERVO_READ  // 0x1A00 TxPDO
    {
        uint16  StatusWord;                 // 0x6041
        int8    ModeOfOperationDisplay;     // 0x6061
        int32   PositionActualValue;        // 0x6064
        int32   VelocityActualValue;        // 0x606C
        int16   TorqueActualValue;          // 0x6077
        int32   current_actual_value;       // 0x30D1
        // uint32   DigitalInput;               // 0x60FD
        // uint16   ErrorCode;                  // 0x603F
    };
};
#pragma pack(pop)
///////////////////////////////////////////////

#define EPOS4_NUM 1

#define CONTROL_PERIOD_IN_ms  ( 1.0 )  // ms
#define CONTROL_PERIOD_IN_s   ( 1e-03 * CONTROL_PERIOD_IN_ms )
#define CONTROL_PERIOD_IN_us  ( 1e03 * CONTROL_PERIOD_IN_ms )
#define CONTROL_PERIOD_IN_ns  ( 1e06 * CONTROL_PERIOD_IN_ms )

#endif // ECATDEF_HPP
