/**
 * @file init.cpp
 *
 */


#include <Arduino.h>
#include <oscc_pid.h>

#include "can_protocols/fault_can_protocol.h"
#include "can_protocols/steering_can_protocol.h"
#include "communications.h"
#include "debug.h"
#include "globals.h"
#include "init.h"
#include "oscc_can.h"
#include "oscc_timer.h"
#include "vehicles.h"

void init_globals( void )
{
    g_steering_control_state.enabled = false;
    g_steering_control_state.operator_override = false;
    g_steering_control_state.dtcs = 0;
    g_steering_pid.min_control = -0.24;
    g_steering_pid.max_control = 0.24;
    g_steering_pid.proportional_gain = 0.08;
    g_steering_pid.derivative_gain = 0.01;
    g_steering_pid.integral_gain = 0.1;
    g_steering_pid.windup_guard = 2.0;
    g_steering_pid.ff_gain = 0.0;
    g_steering_pid.filter_rate = 0.95;
    pid_zeroize(&g_steering_pid, g_steering_pid.windup_guard);
    curr_angle = 0.0;
    setpoint = 0.0;
    new_data = false;
}


void init_devices( void )
{
    pinMode( PIN_DAC_CHIP_SELECT, OUTPUT );
    pinMode( PIN_TORQUE_SENSOR_HIGH, INPUT );
    pinMode( PIN_TORQUE_SENSOR_LOW, INPUT );
    pinMode( PIN_TORQUE_SPOOF_HIGH, INPUT );
    pinMode( PIN_TORQUE_SPOOF_LOW, INPUT );
    pinMode( PIN_SPOOF_ENABLE, OUTPUT );

    cli();
    digitalWrite( PIN_DAC_CHIP_SELECT, HIGH );
    digitalWrite( PIN_SPOOF_ENABLE, LOW );
    sei();
}


void init_communication_interfaces( void )
{
    #ifdef DEBUG
    init_serial( );
    #endif

    DEBUG_PRINT( "init Control CAN - " );
    init_can( g_control_can );

    // Filter CAN messages - accept if (CAN_ID & mask) == (filter & mask)
    // Set buffer 0 to filter only steering module and global messages
    g_control_can.init_Mask( 0, 0, 0x7F0 ); // Filter for 0x0N0 to 0x0NF
    g_control_can.init_Filt( 0, 0, OSCC_STEERING_CAN_ID_INDEX );
    g_control_can.init_Filt( 1, 0, OSCC_FAULT_CAN_ID_INDEX );
    g_control_can.init_Filt( 3, 0, KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID);
    // Accept only CAN Disable when buffer overflow occurs in buffer 0
    g_control_can.init_Mask( 1, 0, 0x7FF ); // Filter for one CAN ID
    g_control_can.init_Filt( 2, 1, OSCC_STEERING_DISABLE_CAN_ID );
}


void start_timers( void )
{
    timer1_init( OSCC_REPORT_STEERING_PUBLISH_FREQ_IN_HZ, publish_steering_report );
    timer2_init(50, update_steering_pid);

}
