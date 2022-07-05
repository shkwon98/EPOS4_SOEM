#ifndef __CECATMASTER_H__
#define __CECATMASTER_H__

#include <iostream>
#include <sys/mman.h>
#include <thread>
// #include <pthread.h>
// #include <cstdlib>
// #include <sys/time.h>
// #include <unistd.h>
// #include <sched.h>
// #include <time.h>
// #include <mutex>
#include "ethercat.h"
// #include "scheduling.h"

namespace ecat
{
    /**
     * This class abstracts the role of the master in an ethercat network.
     * It implements the effective communication between master and slaves.
    */
    class CEcatMaster
    {
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

    public:
        /**
         * Constructor of Master
         * @param char* ifname this is the port for ethercat comunication
         */
        CEcatMaster(char *ifname);

        /**
         * Distructor
        */
        ~CEcatMaster();

        /**
         * Initialize the peripheral for the communication.
         * @param char* ifname this is the port for ethercat comunication
         * @throw runtime_error
        */
        void initialize(char *ifname);

        /**
         * Configuration and initialization of slave.
         * @return int number_slave returns the number of slaves found
         * @throw runtime_error
        */
        int config_init();

        /**
         * It configures the IOmap.
         * @throw runtime_error
        */
        void configMap();

        /**
         * Configure DC mechanism
         * @return bool true if the DC mechanism is setted.
         * @throw runtime_error
        */
        bool configDC();

        /**
         * Changes the status of a slave.
         * @param int slave this is the position of the slave in the network.
         * @param uint16 state this is the state in which you want to move the slave.
         * @param int timeout
         * @return uint16 written state returns the state written by the master
         * @throw runtime_error
        */
        uint16 movetoState(uint16 slave, int state, int timeout);

        uint8 *getOutput_slave(uint16 position);

        uint8 *getInput_slave(uint16 position);

        void sendAndReceive(int timeout);

        void config_ec_sync0(uint16 position, bool activate, uint32 cycletime, int cycleshift);

        /**
         * Print the state of all slaves.
        */
        void printState();

        /**
         * It moves all slaves to the PRE_OP state and closes the ethercat connection.
         * @param int timeout
        */
        void close_master();







        /**
         * Creates the real-time thread for the cyclical exchange of ethercat package.
         * It sends the buffer IOmap every 'cycletime'
         * @param int timeout
         * @param int64 cycletime this is the time of cycle to send ethercat package.
        */
        void createThread(int64 cycleTime);

        /**
         * It sets the time of cycle used by a real-time thread to exchange ethercat package thorow IOmap buffer.
         * @param uint64 cycletime this is the time of cycle to send ethercat package.
        */
        void setCycle(int64 cycletime);














        // /**
        //  * When the slave move from PRE_OP state to SAFE_OP state, it calls slave.setup to set parameters and map PDO.
        //  * @param int slave this is the position of the slave in the network.
        //  * @param (int)(*setup)(uint16 x) this is a function pointer.
        // */
        // void setupSlave(int slave, int (*setup)(uint16 position));





        // void waitThread();

        // /**
        //  * Each slave has to call this method to access the IObuffer.
        // */
        // void mutex_down();

        // /**
        //  * Each slave has to call this method to release the IObuffer.
        // */
        // void mutex_up();
    };
}

#endif // __CECATMASTER_H__