#ifndef CECATCOMMAND_HPP
#define CECATCOMMAND_HPP

#include <sys/mman.h>
#include <iostream>
// #include <thread>
// #include <pthread.h>
// #include <cstdlib>
// #include <sys/time.h>
// #include <unistd.h>
// #include <sched.h>
// #include <time.h>
// #include <mutex>
#include "ethercat.h"
// #include "scheduling.h"


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

// 0x1600 RxPDO
typedef struct PACKED
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
}SERVO_WRITE;

// 0x1A00 TxPDO
typedef struct PACKED
{
    uint16  StatusWord;                 // 0x6041
    int8    ModeOfOperationDisplay;     // 0x6061
    int32   PositionActualValue;        // 0x6064
    int32   VelocityActualValue;        // 0x606C
    int16   TorqueActualValue;          // 0x6077
    int32   current_actual_value;       // 0x30D1
    // uint32   DigitalInput;               // 0x60FD
    // uint16   ErrorCode;                  // 0x603F
}SERVO_READ;


// typedef struct _SERVO_IO
// {
//     SERVO_WRITE    OutParam;
//     SERVO_READ     InParam;
// }SERVO_IO;

typedef struct _SERVO_IO_pt
{
    SERVO_WRITE    *writeParam;
    SERVO_READ     *readParam;
}SERVO_IO_pt;



namespace ecat
{
    /**
     * This class abstracts the role of the master in an ethercat network.
     * It implements the effective communication between master and slaves.
    */
    class CEcatMaster
    {
    public:
        CEcatMaster(char *ifname);
        ~CEcatMaster();

        /**
         * Initialize the peripheral for the communication.
         * @param char* ifname this is the port for ethercat comunication
        */
        void initialize(char *ifname);

        /**
         * Configuration and initialization of slave.
         * @return int number_slave returns the number of slaves found
        */
        int config_init();

        /**
         * When the slave move from PRE_OP state to SAFE_OP state, it calls slave.setup to set parameters and map PDO.
         * @param int slave this is the position of the slave in the network.
         * @param (int)(*setup)(uint16 slaveIdx) this is a function pointer.
        */
        void setupPDO(int slave, int (*setup)(uint16 slaveIdx));

        /**
         * It configures the IOmap.
        */
        void configMap();

        /**
         * Configure DC mechanism
         * @return bool true if the DC mechanism is setted.
        */
        bool configDC();

        /**
         * Changes the status of a slave.
         * @param uint16 slave this is the position of the slave in the network.
         * @param uint16 state this is the state in which you want to move the slave.
         * @param int timeout
         * @return uint16 written state returns the state written by the master
        */
        uint16 movetoState(uint16 slave, uint16 state, int timeout);

        void map_structs(SERVO_IO_pt servo[], int num_of_servos);

        void sendAndReceive(int timeout);

        void config_ec_sync0(uint16 slave, bool activate, uint32 cycletime, int cycleshift);

        void printState1();
        void printState2();

        /**
         * It closes the ethercat connection.
        */
        void close_master();


        // /**
        //  * Creates the real-time thread for the cyclical exchange of ethercat package.
        //  * It sends the buffer IOmap every 'cycletime'
        //  * @param int timeout
        //  * @param int64 cycletime this is the time of cycle to send ethercat package.
        // */
        // void createThread(int64 cycleTime);

        // /**
        //  * It sets the time of cycle used by a real-time thread to exchange ethercat package thorow IOmap buffer.
        //  * @param uint64 cycletime this is the time of cycle to send ethercat package.
        // */
        // void setCycle(int64 cycletime);


        // void waitThread();

        // /**
        //  * Each slave has to call this method to access the IObuffer.
        // */
        // void mutex_down();

        // /**
        //  * Each slave has to call this method to release the IObuffer.
        // */
        // void mutex_up();

    private:
        char IOmap[4096]; /**< Buffer used by all slaves to write and read data */
        // pthread_t tidm;
        // bool thread = false;
        // int64 cycletime;
        // int64 toff;
        // bool shutdown = true;
        // int i = 0;
        // long int time1;
        // long int time2;
        // std::array<long int, 50000> timecycle;
        // long int cycle;
        // void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);
        // void ecatthread();
        // void add_timespec(struct timespec *ts, int64 addtime);
        // std::thread control_thread;
        // std::mutex mtx;
    };
}  // namespace ecat

#endif // CECATCOMMAND_HPP
