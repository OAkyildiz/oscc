/**
 * @file oscc_pid.cpp
 *
 */


#include "oscc_pid.h"
#include "vehicles.h"
#include "debug.h"

void pid_zeroize( pid_s* pid, float integral_windup_guard )
{
    // set prev and integrated error to zero
    pid->prev_input = 0;
    pid->int_error = 0;
    pid->prev_steering_angle = 0;
    pid->windup_guard = integral_windup_guard;
}


int pid_update( pid_s* pid, float setpoint, float input, float dt, float ff, bool debug)
{
    float diff;
    float p_term;
    float i_term;
    float d_term;
    float ff_term;

    float curr_error = setpoint - input;

    static int count = 0;

    if( dt <= 0 )
    {
        return PID_ERROR;
    }

    // integration with windup guarding
    pid->int_error += (curr_error * dt);

    count++;

    if (pid->int_error < -(pid->windup_guard))
    {
        pid->int_error = -(pid->windup_guard);
    }
    else if (pid->int_error > pid->windup_guard)
    {
        pid->int_error = pid->windup_guard;
    }

    // differentiation
    diff = ((input - pid->prev_input) / dt);

    // scaling
    p_term = (pid->proportional_gain * curr_error);
    i_term = (pid->integral_gain     * pid->int_error);
    d_term = (pid->derivative_gain   * diff);
    ff_term = (pid->ff_gain * ff);
    // summation of terms
    pid->control = CONSTRAIN(p_term + i_term - d_term, pid->min_control + ff_term, pid->max_control);
    pid->filtered_control = pid->filter_rate * pid->filtered_control + (1-pid->filter_rate) * pid->control;
    // save current error as previous error for next iteration
    pid->prev_input = input;


    if(debug) {
        int dt = (int)(dt*1000);
        DEBUG_PRINT(dt);
        DEBUG_PRINT(",");
        DEBUG_PRINT(setpoint);
        DEBUG_PRINT(",");
        DEBUG_PRINT(input);
        DEBUG_PRINT(",");
        DEBUG_PRINT(pid->control);
        DEBUG_PRINT("::");
        DEBUG_PRINT(p_term);
        DEBUG_PRINT("-");
        DEBUG_PRINT(i_term);
        DEBUG_PRINT("-");
        DEBUG_PRINT(d_term);
        DEBUG_PRINT("-");
        DEBUG_PRINT(ff_term);
        DEBUG_PRINT("::");
        DEBUG_PRINTLN(pid->prev_steering_angle);
    }

    return PID_SUCCESS;
}
