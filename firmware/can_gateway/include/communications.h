/**
 * @file communications.h
 * @brief Communication functionality.
 *
 */


#ifndef _OSCC_CAN_GATEWAY_COMMUNICATIONS_H_
#define _OSCC_CAN_GATEWAY_COMMUNICATIONS_H_


#include "globals.h"


// ****************************************************************************
// Function:    check_for_module_reports
//
// Purpose:     Checks Control CAN bus for reports from the modules.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void check_for_module_reports( void );


// ****************************************************************************
// Function:    republish_obd_frames_to_control_can_bus
//
// Purpose:     Republish pertinent frames on the OBD CAN bus to the Control CAN
//              bus.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void republish_obd_frames_to_control_can_bus( void );
void process_wheel_speed(uint8_t * data);
void process_velocity_trajectory(uint8_t * data);
void send_throttle( float throttle );
void send_brake( float brake );

#endif /* _OSCC_CAN_GATEWAY_COMMUNICATIONS_H_ */
