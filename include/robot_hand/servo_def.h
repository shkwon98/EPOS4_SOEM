#ifndef __SERVO_DEF_H__
#define __SERVO_DEF_H__

#include <stdint.h>

#define _BV(bit) 				(1 << (bit)) 
#define bit_is_set(val, bit) 	(val & _BV(bit))
#define bit_is_clear(val, bit) 	(!(val & _BV(bit)))
#define sbi(val,bit) 			((val) |= _BV(bit)) 
#define cbi(val,bit) 			((val) &= ~_BV(bit))

#define STATUSWORD_READY_TO_SWITCH_ON_BIT 		0
#define STATUSWORD_SWITCHED_ON_BIT 				1
#define STATUSWORD_OPERATION_ENABLE_BIT 		2
#define STATUSWORD_FAULT_BIT 					3
#define STATUSWORD_VOLTAGE_ENABLE_BIT 			4
#define STATUSWORD_QUICK_STOP_BIT 				5
#define STATUSWORD_SWITCH_ON_DISABLE_BIT 		6
#define STATUSWORD_NO_USED_WARNING_BIT 			7
#define STATUSWORD_ELMO_NOT_USED_BIT 			8
#define STATUSWORD_REMOTE_BIT 					9
#define STATUSWORD_TARGET_REACHED_BIT 			10
#define STATUSWORD_INTERNAL_LIMIT_ACTIVE_BIT	11
#define STATUSWORD_HOMING_ATTAINED_BIT	        12

/**
 * Control CoE FSM(Finite State Machine)
 */
#define EC_CONTROL_COMMAND					        0x02
#define CONTROL_COMMAND_DIABLE_VOLTAGE		        0x0000
#define CONTROL_COMMAND_QUICK_STOP		        	0x0002
#define CONTROL_COMMAND_SHUTDOWN		        	0x0006
#define CONTROL_COMMAND_SWITCH_ON		        	0x0007
#define CONTROL_COMMAND_ENABLE_OPERATION        	0x000F
#define CONTROL_COMMAND_SWITCH_ON_ENABLE_OPERATION	0x000F
#define CONTROL_COMMAND_DISABLE_OPERATION	        0x0007
#define CONTROL_COMMAND_FAULT_RESET			        0x0080

/**
 * Operation mode
 */
#define EC_OP_MODE							0x03

#define OP_MODE_NO_MODE						0x00
#define OP_MODE_PROFILE_POSITION			0x01
#define OP_MODE_VELOCITY					0x02
#define OP_MODE_PROFILE_VELOCITY			0x03
#define OP_MODE_TORQUE_PROFILE				0x04
#define OP_MODE_HOMING						0x06
#define OP_MODE_INTERPOLATED_POSITION		0x07
#define OP_MODE_CYCLIC_SYNC_POSITION		0x08
#define OP_MODE_CYCLIC_SYNC_VELOCITY		0x09
#define OP_MODE_CYCLIC_SYNC_TORQUE			0x0A

#define SERVO_TIMEOUT						50000

typedef int32_t  INT32;
typedef uint32_t UINT32;
typedef int16_t  INT16;
typedef uint16_t UINT16;
typedef uint8_t UINT8;
typedef int8_t INT8;

#ifndef PI
#define PI	    3.14159265359
#define PI2	    6.28318530718
#define RAD2DEG		57.2957795131
#define DEG2RAD		0.01745329252
#endif

#ifdef __cplusplus
extern "C" {
#endif
    int ServoOn_GetCtrlWrd(uint16_t StatusWord, uint16_t *ControlWord);
#ifdef __cplusplus
}
#endif

#endif // __SERVO_DEF_H__