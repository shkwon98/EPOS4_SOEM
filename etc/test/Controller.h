#include "Meca500.h"
#include "ATINano43.h"
#include <pthread.h>

namespace sun
{
    class Controller
    {
    private:
        Meca500 *meca500;
        float gain;
        std::thread thread_controller;


    public:

        Controller(Meca500 *meca500, float gain);

        ~Controller();

        /**
         * Create thread
        */
        void startThread();

        void position_loop_control();

        void waitLoop();

        void start_force_control();
    };
} // namespace sun
