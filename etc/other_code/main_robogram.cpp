#include<stdio.h>
#include<stdlib.h>
#include<malloc.h>
#include<fcntl.h>
#include<errno.h>
#include "SOEM_EPOS.h"

RCMArm leftArm = RCMArm();
RCMArm rightArm = RCMArm();

////////////// PDO Mapping Objects //////////////

//    <0x1601> Receive PDO Mapping 2
//0x60400010 :  Control word                UInt16
//0x60600008 :  Mode of Operation           Int8
//0x60ff0020 :  Target Velocity             Int32
//0x607a0020 :  Target Position             Int32
//0x60810020 :  Profile Velocity            UInt32
//0x60830020 :  Profile acceleration        UInt32
//0x60840020 :  Profile deceleration        UInt32
uint32 rxPDOObjects[7] = { 0x60400010, 0x60600008, 0x60ff0020, 0x607a0020, 0x60810020, 0x60830020, 0x60840020 };
uint8 rxPDOSize = sizeof(rxPDOObjects) / sizeof(rxPDOObjects[0]);


//    <0x1a01> Transmit PDO Mapping 2
//0x60410010 :  Status word                 UInt16
//0x60610008 :  Modes of Operation Display  Int8
//0x60640020 :  Position Actual Value       Int32
//0x606c0020 :  Velocity Actual Value       Int32
//0x60770010 :  Torque Actual Value         Int16
//0x606b0020 :  Position Demand Value       Int32
uint32 txPDOObjects[6] = { 0x60410010, 0x60610008, 0x60640020, 0x606c0020, 0x60770010, 0x60620020 };
uint8 txPDOSize = sizeof(txPDOObjects) / sizeof(txPDOObjects[0]);
//////////////////////////////////////////////////

const char* ifname;

void soem_thread(void *ptr)
{
    PI_data->PI_LX_direction = POSITIVE;
    PI_data->PI_LY_direction = NEGATIVE;
    PI_data->PI_RX_direction = NEGATIVE;
    PI_data->PI_RY_direction = POSITIVE;

    if (SOEM::initializeEtherCAT(ifname))
    {
        // EPOS4 Object Generation
        EPOS4 DISK_Left = EPOS4((char*)CARRI_A_M1, CCW, REVOLUTE, ENCODER_ON_GEAR, REVOLUTE, 262144);
        EPOS4 DISK_Right = EPOS4((char*)CARRI_B_M1, CW, REVOLUTE, ENCODER_ON_GEAR, REVOLUTE, 262144);

        // Check connection of slaves
        DISK_Left.checkConnection();
        DISK_Right.checkConnection();

        // PDO Mapping
        DISK_Left.mapPDO(rxPDOObjects, txPDOObjects, rxPDOSize, txPDOSize);
        DISK_Right.mapPDO(rxPDOObjects, txPDOObjects, rxPDOSize, txPDOSize);

        // Set Position Limit [Deg]. If needed, I'll change Deg to Rad. I used Deg unit because, the safety check function didn't work well with rad unit.
        DISK_Left.setLimit(90, -90, 2);
        DISK_Right.setLimit(90, -90, 3);

        // Turn the motor driver state to Operational
        SOEM::goingOperational();

        DISK_Left.setHomePosition(ABSOLUTE, 46634);
        DISK_Right.setHomePosition(ABSOLUTE, -83538);

        if (SOEM::inOP == true)
        {
            if (SOEM::ecatNumOk && SOEM::ecatWKCOk)
            {
                printf("All slaves Status OK\n");
            }
            else
            {
                printf("Please Check Slave Status\n");
            }

            printf("EPOS4 : Operational state reached for all slaves! Starting in 1 sec \n");
            fflush(stdout);
            usleep(1000000);

            // Get current time from Xenomai realtime timer
            LOOP::getTime();

            while (LOOP::continueLoop())
            {
                if (LOOP::etherCATWKC == true)
                {
                    //////////////////////////////////////////////////////////////////
                    //////////////////////////// Left Arm ////////////////////////////
                    DISK_Left.readState();

                    ///////////////////////////////////////////////////////////////////
                    //////////////////////////// Right Arm ////////////////////////////
                    DISK_Right.readState();

                    rightArm.mCurrentJoint << DISK_Right.mJointValue;

                    if (EPOS4::checkHoming() && PI_Homing)
                    {
                        if (LOOP::loopCounter % 20 == 0)
                        {
                            rightArm.mDeltaJoint = rightArm.rcmstep(CONTROL::Haptic, rightArm.mRCMPoint, rightArm.mCurrentJoint);
                            rightArm.mTargetJoint = rightArm.mCurrentJoint + rightArm.mDeltaJoint;

                            DISK_Right.makeVelocityProfile(rightArm.mDeltaJoint(0), 100, 15, 15); // Insert it to writeTarget after the test is finished
                            Z_Right.makeVelocityProfile(rightArm.mDeltaJoint(1), 100, 15, 15);
                            PI_data->PI_RX_vel = 5;
                            PI_data->PI_RY_vel = 5;
                            RCM_Wrist.makeVelocityProfile(rightArm.mDeltaJoint(4), 100, 15, 15);

                            rightArm.calculatePosition(rightArm.mTargetPosition, rightArm.mTargetJoint);
                        }

                        DISK_Right.convertJointToCnt(rightArm.mTargetJoint(0)); // Insert it to writeTarget after the test is finished
                        Z_Right.convertJointToCnt(rightArm.mTargetJoint(1));
                        PI_data->PI_RX_input = rightArm.mTargetJoint(2);
                        PI_data->PI_RY_input = rightArm.mTargetJoint(3);
                        RCM_Wrist.convertJointToCnt(rightArm.mTargetJoint(4));

                        rightArm.calculatePosition(rightArm.mCurrentPosition, rightArm.mCurrentJoint);
                    }

                    else
                    {
                        DISK_Left.doHoming(1);
                        DISK_Right.doHoming(1);

                        PI_data->PI_LX_input = 0;
                        PI_data->PI_LY_input = 0;
                        PI_data->PI_RX_input = 0;
                        PI_data->PI_RY_input = 0;

                        if (PI_data->PI_LX_homing && PI_data->PI_LY_homing && PI_data->PI_RX_homing && PI_data->PI_RY_homing)
                        {
                            PI_Homing = true;
                        }
                    }

                    DISK_Left.writeTarget();
                    DISK_Right.writeTarget();

                    for (int i = 0; i < 3; i++)
                    {
                        tipPosPub[i] = rightArm.mCurrentPosition(i);
                    }
                }
            }
        }
    }
}

int main(int argc, char *argv[])
{
    pthread_t ecatcheckth;
    pthread_create(&ecatcheckth, NULL, &SOEM::ecatcheck, (void*)&ctime);

    signal(SIGINT, signalCallbackLogger);

    if (argc > 1) ifname = argv[1];

    else
    {
        printf("Usage: simple_test ifname1\nifname = eth0 for example\nEnd program\n");
        return 0;
    }

    rt_task_create(&soem_test_task, "SOEM_Native", 0, 99, 0);
    rt_task_start(&soem_test_task, &soem_thread, NULL);

    while (1) {}

    printf("End program\n");
    return 0;
}