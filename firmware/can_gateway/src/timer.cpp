/**
 * @file timer.cpp
 *
 */


#include "oscc_timer.h"

#include "timer.h"
#include "display.h"
#include "longitudinal_control.h"

void start_timer( void )
{
    timer1_init( DISPLAY_UPDATE_FREQUENCY_IN_HZ, update_display );
    timer2_init( 50, update_long_pid);
}
