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

// #define RAD2DEG     57.2957795131
// #define DEG2RAD     0.01745329252

int servo_enable(uint16 StatusWord, uint16 * ControlWord);


// Structure representing the Object Dictionary
typedef struct _mapping_obj
{
    int32 Index;
    int32 SubIndex;
    int32 Size;
    int32 Value;
}mapping_obj;


//////////////////// PDO Setting ////////////////////
typedef struct PACKED  // 0x1600 RxPDO
{
    int16   J1_TARGET1;         // 0x00060001
    int16   J1_TARGET2;         // 0x00060002
    int16   J1_TARGET3;         // 0x00060003
    int16   J1_TARGET4;         // 0x00060004

    int16   J2_TARGET1;         // 0x00060005
    int16   J2_TARGET2;         // 0x00060006
    int16   J2_TARGET3;         // 0x00060007
    int16   J2_TARGET4;         // 0x00060008

    int16   J3_TARGET1;         // 0x00060009
    int16   J3_TARGET2;         // 0x0006000A
    int16   J3_TARGET3;         // 0x0006000B
    int16   J3_TARGET4;         // 0x0006000C

    int16   J4_TARGET1;         // 0x0006000D
    int16   J4_TARGET2;         // 0x0006000E
    int16   J4_TARGET3;         // 0x0006000F
    int16   J4_TARGET4;         // 0x00060010

    int16   HAND_STATUS_OUT1;   // 0x00060011
    int16   HAND_STATUS_OUT2;   // 0x00060012
    int16   HAND_STATUS_OUT3;   // 0x00060013
    int16   HAND_STATUS_OUT4;   // 0x00060014

    int16   ADD_INFO_1;         // 0x00060015
    int16   ADD_INFO_2;         // 0x00060016
    int16   ADD_INFO_3;         // 0x00060017
    int16   ADD_INFO_4;         // 0x00060018
    int16   ADD_INFO_5;         // 0x00060019
    int16   ADD_INFO_6;         // 0x0006001A
    int16   ADD_INFO_7;         // 0x0006001B
    int16   ADD_INFO_8;         // 0x0006001C
    int16   ADD_INFO_9;         // 0x0006001D
    int16   ADD_INFO_10;        // 0x0006001E
    int16   ADD_INFO_11;        // 0x0006001F
    int16   ADD_INFO_12;        // 0x00060020
}PDO_WRITE;

typedef struct PACKED  // 0x1A00 TxPDO
{
    int16   J1_DATA1;          // 0x00060001
    int16   J1_DATA2;          // 0x00060002
    int16   J1_DATA3;          // 0x00060003
    int16   J1_DATA4;          // 0x00060004

    int16   J2_DATA1;          // 0x00060005
    int16   J2_DATA2;          // 0x00060006
    int16   J2_DATA3;          // 0x00060007
    int16   J2_DATA4;          // 0x00060008

    int16   J3_DATA1;          // 0x00060009
    int16   J3_DATA2;          // 0x0006000A
    int16   J3_DATA3;          // 0x0006000B
    int16   J3_DATA4;          // 0x0006000C

    int16   J4_DATA1;          // 0x0006000D
    int16   J4_DATA2;          // 0x0006000E
    int16   J4_DATA3;          // 0x0006000F
    int16   J4_DATA4;          // 0x00060010

    int16   HAND_STATUS_IN1;   // 0x00060011
    int16   HAND_STATUS_IN2;   // 0x00060012
    int16   HAND_STATUS_IN3;   // 0x00060013
    int16   HAND_STATUS_IN4;   // 0x00060014

    int16   ADD_INFO_1;         // 0x00060015
    int16   ADD_INFO_2;         // 0x00060016
    int16   ADD_INFO_3;         // 0x00060017
    int16   ADD_INFO_4;         // 0x00060018
    int16   ADD_INFO_5;         // 0x00060019
    int16   ADD_INFO_6;         // 0x0006001A
    int16   ADD_INFO_7;         // 0x0006001B
    int16   ADD_INFO_8;         // 0x0006001C
    int16   ADD_INFO_9;         // 0x0006001D
    int16   ADD_INFO_10;        // 0x0006001E
    int16   ADD_INFO_11;        // 0x0006001F
    int16   ADD_INFO_12;        // 0x00060020
}PDO_READ;

typedef struct PACKED
{
    PDO_WRITE  *write;
    PDO_READ   *read;
}PDO_STRUCT;
/////////////////////////////////////////////////////


#define KISTAR_NUM 1
// #define EPOS4_NUM 1
// #define ELMO_NUM 5

#define TOTAL_SLAVE_NUM ( KISTAR_NUM )
// TOTAL_SLAVE_NUM ( EPOS4_NUM + ELMO_NUM )

#define CONTROL_PERIOD_IN_ms  ( 1.0 )  // ms
#define CONTROL_PERIOD_IN_s   ( 1e-03 * CONTROL_PERIOD_IN_ms )
#define CONTROL_PERIOD_IN_us  ( 1e03 * CONTROL_PERIOD_IN_ms )
#define CONTROL_PERIOD_IN_ns  ( 1e06 * CONTROL_PERIOD_IN_ms )

#endif // ECATDEF_HPP
