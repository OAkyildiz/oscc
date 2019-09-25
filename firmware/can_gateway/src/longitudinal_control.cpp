//
// Created by faraz on 3/2/19.
//

#include <display.h>
#include <can_protocols/throttle_can_protocol.h>
#include <oscc_pid.h>
#include "longitudinal_control.h"
#include "globals.h"
#include "oscc_pid.h"
#include "debug.h"
#include "communications.h"

#define TRAJECTORY_DELTA_MILLIS 250.0

void update_long_pid() {
  if (g_new_data)
  {
//    DEBUG_PRINTLN("NEW DATA");
    g_new_data = false;

    float delta_t_sec;
    unsigned long curr_time = millis();
    delta_t_sec = (curr_time - g_last_long_update_time)/1000.0;
    float delta_plan_life_ms = millis() - g_last_trajectory_update_time;
    float target_speed = interpolate(g_speed_trajectory.target_vel_start, g_speed_trajectory.target_vel_start, TRAJECTORY_DELTA_MILLIS, delta_plan_life_ms);
    float target_acc = interpolate(g_speed_trajectory.target_acc_start, g_speed_trajectory.target_acc_start, TRAJECTORY_DELTA_MILLIS, delta_plan_life_ms);
    if (g_display_state.status_screen.throttle == MODULE_STATUS_ENABLED && g_display_state.status_screen.brakes == MODULE_STATUS_ENABLED) {
        pid_update(&g_long_pid, target_speed, g_curr_speed, delta_t_sec, target_acc, true);
      send_control(g_long_pid.filtered_control);
    } else {
      send_control(0.0);
      pid_zeroize(&g_long_pid, g_long_pid.windup_guard);
    }

    g_last_long_update_time = curr_time;
  }
}

float interpolate(float start, float end, float max, float interp) {
  interp = min(interp, max);
  return start + (end - start) / max * interp;
}

void send_control(float control) {
  if(control > 0) {
    send_throttle(control);
  } else {
    send_brake(abs(control));
  }
}
