#include "pch.h"
#include <stdio.h>
#include <iostream>
#include <signal.h>
#include <strings.h>

#include "hal.h"
#include "Dynamixel.h"
#include "RX28.h"

using namespace std;

static bool bDone = false;
static bool bRunning = false;

static void quit(int /*sig*/)
{
    bDone = true;
    if ( !bRunning )	// if we aren't running yet, we are still configuring, so terminate
        exit(0);
}

struct DynamixelDevice
{
    hal_float_t input_scale;
    hal_float_t output_scale;
    hal_float_t *velocity_cmd;
    hal_float_t *position_cmd;
    hal_float_t *torque_limit;
    hal_bit_t *enable;
    hal_s32_t *raw_position_fb;
    hal_float_t *position_fb;
    hal_float_t *velocity_fb;
    hal_float_t *torque_fb;
};

struct hal_data_t
{
    hal_u32_t timeout_errors;
    hal_u32_t checksum_errors;
    hal_u32_t data_errors;
    hal_float_t updates_per_second;

    DynamixelDevice devices[0];
} *halData;


static uint16_t MakeDynamixelVelocity(hal_float_t vel)
{
    if ( vel > 1.0 )
        vel = 1.0;
    else if (vel < -1.0 )
        vel = -1.0;
    int16_t speed = (int16_t)((vel<0?-vel:vel) * 1023);
    if ( vel < 0 )
        speed |= 0x400;
    return speed;
}

int main(int argc, char *argv[])
{
    const string moduleName = "dynamixelcomp";
    int retval = -1;

    // read the commandline arguments
    int baud = 0;
    string device;
    vector<int> ids;
    for ( int i = 1; i < argc; i++ )
    {
        string arg(argv[i]);
        fprintf(stderr,"arg=%s\n", arg.c_str());

        if ( strcasecmp(arg.substr(0,5).c_str(), "baud=") == 0 )
        {
            baud = atoi(arg.substr(5).c_str());
            fprintf(stderr, "%s: baud = %d\n", moduleName.c_str(), baud);
        }
        else if ( strcasecmp(arg.substr(0,7).c_str(), "device=") == 0 )
        {
            device = arg.substr(7);
            fprintf(stderr, "%s: device = %s\n", moduleName.c_str(), device.c_str());
        }
        else if ( strcasecmp(arg.substr(0,4).c_str(), "ids=") == 0 )
        {
            string idList = arg.substr(4);
            string::size_type last = 0;
            string::size_type n = 0;
            while ( n != string::npos )
            {
                n = idList.find(',',last);
                string s;
                if ( n != string::npos )
                {
                    s = idList.substr(last,n-last);
                    n++;
                }
                else
                {
                    s = idList.substr(last);
                }
                ids.push_back(atoi(s.c_str()));
                last = n;
            }

            fprintf(stderr, "%s: %d servos\n", moduleName.c_str(), ids.size());
            for ( int id = 0; id < ids.size(); id++ )
                fprintf(stderr, "%s: id%d = %d\n", moduleName.c_str(), id, ids[id] );
        }
    }
    const unsigned int device_count = ids.size();

    // All ready to go.
    signal(SIGINT, quit);
    signal(SIGTERM, quit);

    int hal_comp_id = hal_init( moduleName.c_str() );
    if ((hal_comp_id < 0) || bDone)
    {
        fprintf( stderr, "%s: ERROR: hal_init failed: %d", moduleName.c_str(), hal_comp_id );
        return hal_comp_id;
    }


    /* grab some shmem to store the HAL data in */
    halData = (hal_data_t *)hal_malloc( sizeof(hal_data_t) + device_count * sizeof(DynamixelDevice) );
    if ((halData == 0) || bDone) {
        fprintf( stderr, "%s: ERROR: unable to allocate shared memory", moduleName.c_str());
        retval = -1;
        return -1;
    }

    // Create the hal pins
    for (unsigned int i = 0; i < device_count; i++ )
    {
        int nRet = hal_pin_float_newf( HAL_IN, &(halData->devices[i].velocity_cmd), hal_comp_id, "%s.%d.velocity-cmd", moduleName.c_str(), ids[i]);
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "velocity-cmd", ids[i] );
            retval = nRet;
            throw;
        }
        *(halData->devices[i].velocity_cmd) = 0;
        nRet = hal_pin_float_newf( HAL_OUT, &(halData->devices[i].position_fb), hal_comp_id, "%s.%d.position-fb", moduleName.c_str(), ids[i] );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "position-fb", ids[i] );
            retval = nRet;
            throw;
        }
        nRet = hal_param_float_newf( HAL_RW, &(halData->devices[i].input_scale), hal_comp_id, "%s.%d.input-scale", moduleName.c_str(), ids[i] );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_param_float_newf failed for param '%s': %d", moduleName.c_str(), "input-scale", ids[i] );
            retval = nRet;
            throw;
        }
        halData->devices[i].input_scale = 1.0;
        nRet = hal_param_float_newf( HAL_RW, &(halData->devices[i].output_scale), hal_comp_id, "%s.%d.output-scale", moduleName.c_str(), ids[i] );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_param_float_newf failed for param '%s': %d", moduleName.c_str(), "output-scale", ids[i] );
            retval = nRet;
            throw;
        }
        halData->devices[i].output_scale = 1.0;
        nRet = hal_pin_s32_newf( HAL_OUT, &(halData->devices[i].raw_position_fb), hal_comp_id, "%s.%d.raw-position-fb", moduleName.c_str(), ids[i] );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "raw-position-fb", ids[i] );
            retval = nRet;
            throw;
        }
        nRet = hal_pin_float_newf( HAL_IN, &(halData->devices[i].position_cmd), hal_comp_id, "%s.%d.position-cmd", moduleName.c_str(), ids[i] );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "position-cmd", ids[i] );
            retval = nRet;
            throw;
        }
        nRet = hal_pin_float_newf( HAL_IN, &(halData->devices[i].torque_limit), hal_comp_id, "%s.%d.torque-limit", moduleName.c_str(), ids[i] );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "torque-limit", ids[i] );
            retval = nRet;
            throw;
        }
        nRet = hal_pin_float_newf( HAL_OUT, &(halData->devices[i].torque_fb), hal_comp_id, "%s.%d.torque-fb", moduleName.c_str(), ids[i] );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "torque-fb", ids[i] );
            retval = nRet;
            throw;
        }
        nRet = hal_pin_float_newf( HAL_OUT, &(halData->devices[i].velocity_fb), hal_comp_id, "%s.%d.velocity-fb", moduleName.c_str(), ids[i] );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "velocity-fb", ids[i] );
            retval = nRet;
            throw;
        }
         nRet = hal_pin_bit_newf( HAL_IN, &(halData->devices[i].enable), hal_comp_id, "%s.%d.enable", moduleName.c_str(), ids[i] );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "enable", ids[i] );
            retval = nRet;
            throw;
        }
        *(halData->devices[i].enable) = 0;
    }
    int nRet = hal_param_u32_newf( HAL_RW, &(halData->checksum_errors), hal_comp_id, "%s.checksum-errors", moduleName.c_str() );
    if ( nRet != 0 )
    {
        fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s'", moduleName.c_str(), "checksum-errors");
        retval = nRet;
        throw;
    }
    halData->checksum_errors = 0;
    nRet = hal_param_u32_newf( HAL_RW, &(halData->timeout_errors), hal_comp_id, "%s.timeout-errors", moduleName.c_str() );
    if ( nRet != 0 )
    {
        fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s'", moduleName.c_str(), "timeout-errors");
        retval = nRet;
        throw;
    }
    halData->timeout_errors = 0;
    nRet = hal_param_u32_newf( HAL_RW, &(halData->data_errors), hal_comp_id, "%s.data-errors", moduleName.c_str() );
    if ( nRet != 0 )
    {
        fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s'", moduleName.c_str(), "data-errors");
        retval = nRet;
        throw;
    }
    halData->data_errors = 0;
    nRet = hal_param_float_newf( HAL_RW, &(halData->updates_per_second), hal_comp_id, "%s.updates-per-second", moduleName.c_str() );
    if ( nRet != 0 )
    {
        fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s'", moduleName.c_str(), "updates-per-second");
        retval = nRet;
        throw;
    }
    halData->updates_per_second = 0;

    // ready
    hal_ready( hal_comp_id );

    bRunning = true;

    Dynamixel dmx(device.c_str(),baud);
    for ( int i = 0; i < device_count; i++ )
    {
        RX28 *rx = new RX28(ids[i]);
        dmx.addDevice(rx);
    }

    if (!dmx.open())
    {
        fprintf(stderr, "Failed to open port\n");
        throw;
    }
    dmx.enableTorque();
    dmx.setWheelMode();
    //dmx.goalAcceleration(254);
    //dmx.enableTorque(true);

    time_t old_t = time(NULL);
    unsigned int loopCount = 0;
    while ( !bDone )
    {
        time_t t = time(NULL);
        if ( t != old_t )
        {
            old_t = t;
            halData->updates_per_second = loopCount;
            loopCount = 0;
        }
        bool enableChanged = false;
        for ( unsigned int i = 0; i < dmx.devices.size(); i++ )
        {
            if ( *(halData->devices[i].enable) != dmx.devices[i]->enable )
            {
                dmx.devices[i]->enable = *(halData->devices[i].enable);
                enableChanged = true;
            }
        }

        if ( enableChanged )
        {
            dmx.enableTorque();
        }

        halData->checksum_errors = dmx.checksumErrors;
        halData->data_errors = dmx.dataErrors;
        halData->timeout_errors = dmx.timeoutErrors;

        dmx.readPositions();
        for ( unsigned int i = 0; i < dmx.devices.size(); i++ )
        {
            *(halData->devices[i].raw_position_fb) = dmx.devices[i]->position_fb;
            *(halData->devices[i].position_fb) = dmx.devices[i]->position_fb / halData->devices[i].output_scale;
            *(halData->devices[i].velocity_fb) = dmx.devices[i]->velocity_fb;
            *(halData->devices[i].torque_fb) = dmx.devices[i]->torque_fb;
        }

        for ( unsigned int i = 0; i < dmx.devices.size(); i++ )
        {
            dmx.devices[i]->SetVelocity( MakeDynamixelVelocity(*(halData->devices[i].velocity_cmd) / halData->devices[i].input_scale) );
            dmx.devices[i]->SetTorqueLimit( *(halData->devices[i].torque_limit ) );
            dmx.devices[i]->SetPosition( *(halData->devices[i].position_cmd ) );
        }
        dmx.Update();

        loopCount++;
    }

    for ( unsigned int i = 0; i < dmx.devices.size(); i++ )
        dmx.devices[i]->enable = false;
    dmx.enableTorque();

    dmx.close();

    hal_exit( hal_comp_id );
    return retval;

}


/*

TODO
   - add other pins, eg LED
   - parse command line for configuration
        - baud 3000000
        - tty device /dev/ttyUSB0
        - device ids 2,3 (and names 0,1?)

*/
