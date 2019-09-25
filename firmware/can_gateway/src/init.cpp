/**
 * @file init.cpp
 *
 */


#include "debug.h"
#include "globals.h"
#include "init.h"
#include "oscc_can.h"
#include "oscc_serial.h"
#include <oscc_pid.h>

void init_globals( void )
{
    memset(
        &g_display_state,
        0,
        sizeof(g_display_state) );
    g_long_pid.min_control = -0.4;
    g_long_pid.max_control = 0.4;
    g_long_pid.proportional_gain = 0.1;
    g_long_pid.derivative_gain = 0.00;
    g_long_pid.integral_gain = 0.03;
    g_long_pid.windup_guard = 3.0;
    g_long_pid.ff_gain = 0.14;
    g_long_pid.filter_rate = 0.95;
    g_last_trajectory_update_time = 0.0;
    pid_zeroize(&g_long_pid, g_long_pid.windup_guard);
    g_curr_speed = 0.0;
    memset(&g_speed_trajectory, 0, sizeof(g_speed_trajectory));
    g_new_data = false;
}


void init_communication_interfaces( void )
{
    #ifdef DEBUG
    init_serial();
    #endif

    DEBUG_PRINTLN( "init display");
    init_display( );

    DEBUG_PRINT( "init OBD CAN - ");
    init_can( g_obd_can );

    DEBUG_PRINT( "init Control CAN - ");
    init_can( g_control_can );
}
