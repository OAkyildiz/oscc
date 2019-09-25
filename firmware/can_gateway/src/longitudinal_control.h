//
// Created by faraz on 3/2/19.
//

#ifndef FIRMWARE_LONGITUDINAL_CONTROL_H
#define FIRMWARE_LONGITUDINAL_CONTROL_H

void update_long_pid();
void send_control(float control);
float interpolate(float start, float end, float max, float interp);


#endif //FIRMWARE_LONGITUDINAL_CONTROL_H
