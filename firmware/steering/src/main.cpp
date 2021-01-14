/**
 * @file main.cpp
 *
 */


#include "arduino_init.h"
#include "communications.h"
#include "debug.h"
#include "init.h"
#include "steering_control.h"


int main( void )
{
    init_arduino( );

    init_globals( );

    init_devices( );

    init_communication_interfaces( );

    start_timers( );

    //Serial.begin(115200);
    Serial.println("test");

    DEBUG_PRINTLN( "init complete" );

    while( true )
    {
#ifdef __AVR_ATmega32U4__
        RXLED1;
#endif
        //Serial.println("bark");

        check_for_incoming_message( );

        check_for_faults( );
    }
}
