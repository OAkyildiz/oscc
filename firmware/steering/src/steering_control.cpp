/**
 * @file steering_control.cpp
 *
 */


#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <steering_control.h>
#include <oscc_pid.h>

#include "can_protocols/steering_can_protocol.h"
#include "communications.h"
#include "debug.h"
#include "dtc.h"
#include "globals.h"
#include "oscc_dac.h"
#include "oscc_check.h"
#include "steering_control.h"
#include "vehicles.h"


unsigned long last_steering_update_time = 0;


static void read_torque_sensor(
    steering_torque_s * value );


static float exponential_moving_average(
    const float alpha,
    const float input,
    const float average );

#ifdef STEERING_OVERRIDE
static uint16_t filtered_diff = 0;
#endif

void check_for_faults( void )
{
    static condition_state_s grounded_fault_state = CONDITION_STATE_INIT;

    steering_torque_s torque;
    read_torque_sensor(&torque);
    g_steering_control_state.torque = STEERING_VOLTS_HIGH_TO_TORQUE(torque.high);

    if ( ( g_steering_control_state.enabled == true )
        || (g_steering_control_state.dtcs > 0) )
    {

#ifdef STEERING_OVERRIDE
        uint16_t unfiltered_diff = abs( ( int )torque.high - ( int )torque.low );

        const float filter_alpha = 0.01;

        if ( filtered_diff == 0 )
        {
            filtered_diff = unfiltered_diff;
        }

        filtered_diff = exponential_moving_average(
            filter_alpha,
            unfiltered_diff,
            filtered_diff);
#endif

        bool inputs_grounded = check_voltage_grounded(
                torque.high,
                torque.low,
                FAULT_HYSTERESIS,
                &grounded_fault_state);

        // sensor pins tied to ground - a value of zero indicates disconnection
        if( inputs_grounded == true )
        {
            disable_control( );

            DTC_SET(
                g_steering_control_state.dtcs,
                OSCC_STEERING_DTC_INVALID_SENSOR_VAL );

            publish_fault_report( );

            DEBUG_PRINTLN( "Bad value read from torque sensor" );
        }
#ifdef STEERING_OVERRIDE
        else if( abs( filtered_diff ) > TORQUE_DIFFERENCE_OVERRIDE_THRESHOLD )
        {
            disable_control( );

            DTC_SET(
                g_steering_control_state.dtcs,
                OSCC_STEERING_DTC_OPERATOR_OVERRIDE );

            publish_fault_report( );

            g_steering_control_state.operator_override = true;

            DEBUG_PRINTLN( "Operator override" );
        }
#endif
        else
        {
            g_steering_control_state.dtcs = 0;

#ifdef STEERING_OVERRIDE
            g_steering_control_state.operator_override = false;
#endif
        }
    }
}


void update_steering(
    uint16_t spoof_command_high,
    uint16_t spoof_command_low )
{
    if ( g_steering_control_state.enabled == true )
    {
        uint16_t spoof_high =
            constrain(
                spoof_command_high,
                STEERING_SPOOF_HIGH_SIGNAL_RANGE_MIN,
                STEERING_SPOOF_HIGH_SIGNAL_RANGE_MAX );

        uint16_t spoof_low =
            constrain(
                spoof_command_low,
                STEERING_SPOOF_LOW_SIGNAL_RANGE_MIN,
                STEERING_SPOOF_LOW_SIGNAL_RANGE_MAX );

        cli();
        g_dac.outputA( spoof_high );
        g_dac.outputB( spoof_low );
        sei();
     }
}


void enable_control( void )
{
    if( g_steering_control_state.enabled == false
        && g_steering_control_state.operator_override == false )
    {
        const uint16_t num_samples = 20;
        prevent_signal_discontinuity(
            g_dac,
            num_samples,
            PIN_TORQUE_SENSOR_HIGH,
            PIN_TORQUE_SENSOR_LOW );

        cli();
        digitalWrite( PIN_SPOOF_ENABLE, HIGH );
        sei();

        g_steering_control_state.enabled = true;

        DEBUG_PRINTLN( "Control enabled" );
    }
}


void disable_control( void )
{
    if( g_steering_control_state.enabled == true )
    {
        const uint16_t num_samples = 20;
        prevent_signal_discontinuity(
            g_dac,
            num_samples,
            PIN_TORQUE_SENSOR_HIGH,
            PIN_TORQUE_SENSOR_LOW );

        cli();
        digitalWrite( PIN_SPOOF_ENABLE, LOW );
        sei();

        g_steering_control_state.enabled = false;

#ifdef STEERING_OVERRIDE
        filtered_diff = 0;
#endif

        DEBUG_PRINTLN( "Control disabled" );
    }
}

static float exponential_moving_average(
    const float alpha,
    const float input,
    const float average )
{
    return ( (alpha * input) + ((1.0 - alpha) * average) );
}

static void read_torque_sensor(
    steering_torque_s * value )
{
    cli();
    value->high = analogRead( PIN_TORQUE_SENSOR_HIGH ) << 2;
    value->low = analogRead( PIN_TORQUE_SENSOR_LOW ) << 2;
    sei();
}

void update_steering_pid() {
  if (new_data)
  {
    new_data = 0;

    float delta_t_sec = 0.0;
    unsigned long curr_time = millis();
    delta_t_sec = (curr_time - last_steering_update_time)/1000.0;

    if (g_steering_control_state.enabled) {
        pid_update(&g_steering_pid, setpoint, curr_angle, delta_t_sec, 0.0);
        apply_torque(g_steering_pid.filtered_control);
    } else {
      apply_torque(0.0);
      pid_zeroize(&g_steering_pid, g_steering_pid.windup_guard);
    }

  last_steering_update_time = curr_time;

    int dt = (int)(delta_t_sec*1000);

    DEBUG_PRINT(dt);
    DEBUG_PRINT(",");
    DEBUG_PRINT(curr_angle);
    DEBUG_PRINT(",");
    DEBUG_PRINT(g_steering_pid.control*10);
    DEBUG_PRINT(",");
    DEBUG_PRINT(g_steering_pid.prev_input);
    DEBUG_PRINT(",");
    DEBUG_PRINT(g_steering_pid.prev_steering_angle);
    DEBUG_PRINT(",");
    DEBUG_PRINTLN(setpoint);
  }
}
