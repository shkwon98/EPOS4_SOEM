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

int servo_enable(uint16 StatusWord, uint16 * ControlWord);


// Structure representing the Object Dictionary
typedef struct _mapping_obj
{
    int32 Index;
    int32 SubIndex;
    int32 Size;
    int32 Value;
}mapping_obj;


typedef struct PACKED
{
    uint16 J0_Servo_On : 1;
    uint16 J1_Servo_On : 1;
    uint16 J2_Servo_On : 1;
    uint16 J3_Servo_On : 1;

    uint16 J4_Servo_On : 1;
    uint16 J5_Servo_On : 1;
    uint16 J6_Servo_On : 1;
    uint16 J7_Servo_On : 1;

    uint16 J8_Servo_On : 1;
    uint16 J9_Servo_On : 1;
    uint16 J10_Servo_On : 1;
    uint16 J11_Servo_On : 1;

    uint16 J12_Servo_On : 1;
    uint16 J13_Servo_On : 1;
    uint16 J14_Servo_On : 1;
    uint16 J15_Servo_On : 1;
}AC_Status1;
typedef struct PACKED
{
    uint16 J0_C_Mode : 1;
    uint16 J1_C_Mode : 1;
    uint16 J2_C_Mode : 1;
    uint16 J3_C_Mode : 1;

    uint16 J4_C_Mode : 1;
    uint16 J5_C_Mode : 1;
    uint16 J6_C_Mode : 1;
    uint16 J7_C_Mode : 1;

    uint16 J8_C_Mode : 1;
    uint16 J9_C_Mode : 1;
    uint16 J10_C_Mode : 1;
    uint16 J11_C_Mode : 1;

    uint16 J12_C_Mode : 1;
    uint16 J13_C_Mode : 1;
    uint16 J14_C_Mode : 1;
    uint16 J15_C_Mode : 1;
}AC_Status2;
typedef struct PACKED
{
    uint16 mode : 4;
    uint16 register_num : 12;
}AC_Status3;
typedef struct PACKED
{
    uint16 NOT_USED : 16;
}AC_Status4;

//////////////////// PDO Setting ////////////////////
typedef struct PACKED  // 0x1600 RxPDO
{
    uint16 JOINT_TARGET[16];
    AC_Status1 Actuator_Status1;
    AC_Status2 Actuator_Status2;
    AC_Status3 Actuator_Status3;
    AC_Status4 Actuator_Status4;
    uint16 ADD_INFO[12];
}PDO_WRITE;
typedef struct PACKED  // 0x1A00 TxPDO
{
    uint16 JOINT_DATA[16];
    AC_Status1 Actuator_Status1;
    AC_Status2 Actuator_Status2;
    AC_Status3 Actuator_Status3;
    AC_Status4 Actuator_Status4;
    uint16 ADD_INFO[12];
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

// #define USE_GUI

#endif // ECATDEF_HPP
