#ifndef SOCKETDEF_HPP
#define SOCKETDEF_HPP

////////////////////////////////////////////////////
/////////////// Socket Configuration ///////////////
////////////////////////////////////////////////////
#define TCP_PORT 2000
#define UDP_PORT 3000
#define GUI_PC_IP "192.168.0.55"
#define RASPBERRY_PI_IP "192.168.0.50"
#define RX_BUFFER_SIZE 1024
#define TX_BUFFER_SIZE 1024
#define PACKET_BUFFER_SIZE 1024


////////////////////////////////////////////////////
///////////////// GUI PC ----> RPI /////////////////
////////////////////////////////////////////////////
#define MODE_OFF                           0x0000
#define MODE_JOINT_POSITION                0x0001
#define MODE_MOTION                        0x0002
// #define COMMAND_RUN_CSP                    0X0001  // Cyclic Synchronous Position Mode
// #define COMMAND_RUN_CSV                    0x0011  // Cyclic Synchronous Velocity Mode
// #define COMMAND_RUN_CST                    0x0021  // Cyclic Synchronous Torque Mode
// #define COMMAND_RUN_PPM                    0x0031  // Profile Position Mode
// #define COMMAND_MODE_STOP_MOTOR           0xFFFF

typedef struct
{
    double    amplitude;
    double    period;
}MOTION_INPUT_LIST;

typedef struct
{
    int16_t   J1_TARGET1;
    int16_t   J1_TARGET2;
    int16_t   J1_TARGET3;
    int16_t   J1_TARGET4;

    int16_t   J2_TARGET1;
    int16_t   J2_TARGET2;
    int16_t   J2_TARGET3;
    int16_t   J2_TARGET4;

    int16_t   J3_TARGET1;
    int16_t   J3_TARGET2;
    int16_t   J3_TARGET3;
    int16_t   J3_TARGET4;

    int16_t   J4_TARGET1;
    int16_t   J4_TARGET2;
    int16_t   J4_TARGET3;
    int16_t   J4_TARGET4;
}JOINT_INPUT_LIST;


////////////////////////////////////////////////////
///////////////// RPI ----> GUI PC /////////////////
////////////////////////////////////////////////////
#define STREAM_MODE            0x0001
// #define

typedef struct
{
    uint32_t timeStamp;

    int16_t   J1_DATA1;
    int16_t   J1_DATA2;
    int16_t   J1_DATA3;
    int16_t   J1_DATA4;

    int16_t   J2_DATA1;
    int16_t   J2_DATA2;
    int16_t   J2_DATA3;
    int16_t   J2_DATA4;

    int16_t   J3_DATA1;
    int16_t   J3_DATA2;
    int16_t   J3_DATA3;
    int16_t   J3_DATA4;

    int16_t   J4_DATA1;
    int16_t   J4_DATA2;
    int16_t   J4_DATA3;
    int16_t   J4_DATA4;
}LOG_DATA;
#define LOG_DATA_SIZE sizeof(LOG_DATA)













// Hardware Specification
// #define LEAD_PITCH 0.25  // 0.25 mm/rev
// #define GEAR_RATIO 17.0  // 17:1

// #define SENSOR_INTER_FACTOR (4.0 * 256.0)

// #define INC_PER_REV GEAR_RATIO*SENSOR_INTER_FACTOR
// #define MM_PER_INC LEAD_PITCH/((float)(GEAR_RATIO*SENSOR_INTER_FACTOR))  // currently 2.45 um/inc
// #define INC_PER_MM (GEAR_RATIO*SENSOR_INTER_FACTOR)/LEAD_PITCH

// #define CONV_INC_to_MM(inc) -1.0*MM_PER_INC*inc
// #define CONV_MM_to_INC(mm)  -1.0*INC_PER_MM*mm

// #define RAD2DEG     57.2957795131
// #define DEG2RAD     0.01745329252

#endif // SOCKETDEF_HPP
