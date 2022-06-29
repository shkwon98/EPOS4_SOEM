#ifndef __PDO_DEF_H__
#define __PDO_DEF_H__

#include "servo_def.h"

// EPOS4 PDO mapping

// Structure representing the Object Dictionary
typedef struct _mapping_obj
{
    int32 Index;
    int32 SubIndex;
    int32 Size;
    int32 Value;
}mapping_obj;

//0x1600 RxPDO
typedef struct PACKED
{
    uint16	ControlWord;		//0x6040
    int8	ModeOfOperation;	//0x6060
    int16   TargetTorque;       //0x6071
    int32	TargetVelocity;		//0x60FF
    int32	TargetPosition;		//0x607A
    uint32  ProfileVelocity;    //0x6081
    //int32	PositionOffset;		//0x60B0
    //int32	VelocityOffset;		//0x60B1
    //int16 TorqueOffset;   	//0x60B2
    //int32   HomePosition;     //0x30B0
    //int8    HomingMethod;     //0x6098
}EPOS4_DRIVE_RxPDO_t;

//0x1A00 TxPDO
typedef struct PACKED
{
    uint16	StatusWord;		        	//0x6041
    int8	ModeOfOperationDisplay;		//0x6061
    int32	PositionActualValue;    	//0x6064
    int32	VelocityActualValue;		//0x606C
    int16	TorqueActualValue;			//0x6077
    int32   current_actual_value;       //0x30D1
    // uint32	DigitalInput;				//0x60FD
    // uint16	ErrorCode;                  //0x603F
}EPOS4_DRIVE_TxPDO_t;


typedef struct _EPOS4_ServoDrive
{
    EPOS4_DRIVE_RxPDO_t 	OutParam;
    EPOS4_DRIVE_TxPDO_t 	InParam;
}EPOS4_ServoDrive_t;

typedef struct _EPOS4_Drive_pt
{
    EPOS4_DRIVE_RxPDO_t 	*ptOutParam;
    EPOS4_DRIVE_TxPDO_t 	*ptInParam;
}EPOS4_Drive_pt;

#endif // __PDO_DEF_H__