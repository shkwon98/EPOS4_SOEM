#ifndef SOCKETDEF_HPP
#define SOCKETDEF_HPP

#include "ethercat.h"

#define TCP_PORT 2000
#define UDP_PORT 3000
#define GUI_PC_IP "192.168.0.9"
#define RASPBERRY_PI_IP "192.168.0.10"

#define RX_BUFFER_SIZE 1024
#define TX_BUFFER_SIZE 1024
#define PACKET_BUFFER_SIZE 1024


typedef enum
{
    RUN_TASK_NONE,
    RUN_TASK_MOTOR_CONTROL
}RUN_TASK_TYPE;

typedef struct
{
    int32   taskType;
    int32   taskMotion;

    float32 disp;
    float32 period;
    int32   repeat;

    // unsigned int numRun;

    // bool footSwitch;
    bool  dataApply;
}TASK_PARAM;

typedef struct
{
    int16      torque;
    int32      velocity;
    int32      position;
    TASK_PARAM taskParam;
}INPUT_LIST;

typedef struct
{
    uint32 timeStamp;

    int32  actualVelocity;
    int16  actualTorque;
    int32  actualPosition;
}LOG_DATA;
#define LOG_DATA_SIZE sizeof(LOG_DATA)



// Hardware Specification
#define LEAD_PITCH 0.25  // 0.25 mm/rev
#define GEAR_RATIO 17.0  // 17:1

#define SENSOR_INTER_FACTOR 4.0*256.0

#define INC_PER_REV GEAR_RATIO*SENSOR_INTER_FACTOR
#define MM_PER_INC LEAD_PITCH/((float)(GEAR_RATIO*SENSOR_INTER_FACTOR))  // currently 2.45 um/inc
#define INC_PER_MM (GEAR_RATIO*SENSOR_INTER_FACTOR)/LEAD_PITCH

#define CONV_INC_to_MM(inc) -1.0*MM_PER_INC*inc
#define CONV_MM_to_INC(mm)  -1.0*INC_PER_MM*mm



////////////////////////////////////////////////////
///////////////// GUI PC ----> RPI /////////////////
////////////////////////////////////////////////////

#define COMMAND_SET_TASK_PARAM             0X0000
#define COMMAND_RUN_CSP                    0X0001  // Cyclic Synchronous Position Mode

#define COMMAND_RUN_CSV                    0x0011  // Cyclic Synchronous Velocity Mode

#define COMMAND_RUN_CST                    0x0021  // Cyclic Synchronous Torque Mode

#define COMMAND_RUN_PPM                    0x0031  // Profile Position Mode

// #define COMMAND_MODE_STOP_MOTOR           0xFFFF



////////////////////////////////////////////////////
///////////////// RPI ----> GUI PC /////////////////
////////////////////////////////////////////////////

#define STREAM_MODE            0x0001
// #define

#endif // SOCKETDEF_HPP
