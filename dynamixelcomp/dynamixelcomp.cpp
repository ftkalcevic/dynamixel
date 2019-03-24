#include "pch.h"
#include <stdio.h>
#include <iostream>
#include <signal.h>

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
    hal_float_t scale;
    hal_s32_t *raw_position;
    hal_float_t *position;
    hal_float_t *velocity;
    hal_bit_t *enable;
};

struct hal_data_t
{
    hal_u32_t timeout_errors;
    hal_u32_t checksum_errors;
    hal_u32_t data_errors;
    hal_float_t updates_per_second;

    DynamixelDevice devices[0];
} *halData;


#define DEVICE_COUNT    2

void update_position(void *, long )
{
}

void update_velocity(void *, long )
{
}

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
    const unsigned int device_count = DEVICE_COUNT;
    int retval = -1;

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
        int nRet = hal_pin_float_newf( HAL_IN, &(halData->devices[i].velocity), hal_comp_id, "%s.%d.velocity", moduleName.c_str(), i);
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "velocity", i );
            retval = nRet;
            throw;
        }
        *(halData->devices[i].velocity) = 0;
        nRet = hal_pin_float_newf( HAL_OUT, &(halData->devices[i].position), hal_comp_id, "%s.%d.position", moduleName.c_str(), i );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "position", i );
            retval = nRet;
            throw;
        }
        nRet = hal_param_float_newf( HAL_RW, &(halData->devices[i].scale), hal_comp_id, "%s.%d.scale", moduleName.c_str(), i );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "scale", i );
            retval = nRet;
            throw;
        }
        halData->devices[i].scale = 1.0;
        nRet = hal_pin_s32_newf( HAL_OUT, &(halData->devices[i].raw_position), hal_comp_id, "%s.%d.raw-position", moduleName.c_str(), i );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "raw-position", i );
            retval = nRet;
            throw;
        }
        nRet = hal_pin_bit_newf( HAL_IN, &(halData->devices[i].enable), hal_comp_id, "%s.%d.enable", moduleName.c_str(), i );
        if ( nRet != 0 )
        {
            fprintf( stderr, "%s: ERROR: hal_pin_new failed for pin '%s': %d", moduleName.c_str(), "enable", i );
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

//    int nRet = hal_export_funct("update-position", update_position, NULL, 1, 0, hal_comp_id);
//    nRet = hal_export_funct("update-velocity", update_velocity, NULL, 1, 0, hal_comp_id);

    // ready
    hal_ready( hal_comp_id );

    bRunning = true;

    Dynamixel dmx("/dev/ttyUSB0",3000000);
    RX28 id2(2);
    RX28 id3(3);
    dmx.addDevice(&id2);
    dmx.addDevice(&id3);
    if (!dmx.open())
    {
        fprintf(stderr, "Failed to open port\n");
        throw;
    }
    dmx.enableTorque();
    dmx.setWheelMode();
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
            *(halData->devices[i].raw_position) = dmx.devices[i]->position;
            *(halData->devices[i].position) = dmx.devices[i]->position / halData->devices[i].scale;
        }

        for ( unsigned int i = 0; i < dmx.devices.size(); i++ )
            dmx.devices[i]->velocity = MakeDynamixelVelocity(*(halData->devices[i].velocity));
        dmx.setVelocities();

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
    - expose error counters
    - add enable pin on each motor (watch enable pin for changes, and then update)
*/
