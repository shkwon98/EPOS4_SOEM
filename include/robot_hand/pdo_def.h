#ifndef __PDO_DEF_H__
#define __PDO_DEF_H__

#include "servo_def.h"

// EPOS4 PDO mapping

// Structure representing the Object Dictionary
typedef struct _mapping_obj
{
    int Index;
    int SubIndex;
    int Size;
    int Value;
} mapping_obj;

//0x1600 RxPDO
typedef struct PACKED
{
    UINT16	ControlWord;		//0x6040
    INT8	ModeOfOperation;	//0x6060
    INT16   TargetTorque;       //0x6071
    INT32	TargetVelocity;		//0x60FF
    INT32	TargetPosition;		//0x607A
    UINT32  ProfileVelocity;    //0x6081
    //INT32	PositionOffset;		//0x60B0
    //INT32	VelocityOffset;		//0x60B1
    //INT16 TorqueOffset;   	//0x60B2
    //INT32   HomePosition;     //0x30B0
    //INT8    HomingMethod;     //0x6098
} EPOS4_DRIVE_RxPDO_t;

//0x1A00 TxPDO
typedef struct PACKED
{
    UINT16	StatusWord;		        	//0x6041
    INT8	ModeOfOperationDisplay;		//0x6061
    INT32	PositionActualValue;    	//0x6064
    INT32	VelocityActualValue;		//0x606C
    INT16	TorqueActualValue;			//0x6077
    INT32   current_actual_value;       //0x30D1
    // UINT32	DigitalInput;				//0x60FD
    // UINT16	ErrorCode;                  //0x603F
} EPOS4_DRIVE_TxPDO_t;


typedef struct _EPOS4_ServoDrive
{
    EPOS4_DRIVE_RxPDO_t 	OutParam;
    EPOS4_DRIVE_TxPDO_t 	InParam;
} EPOS4_ServoDrive_t;

typedef struct _EPOS4_Drive_pt
{
    EPOS4_DRIVE_RxPDO_t 	*ptOutParam;
    EPOS4_DRIVE_TxPDO_t 	*ptInParam;
} EPOS4_Drive_pt;

#endif // __PDO_DEF_H__