/**
 * Beck, 20180513
 * velocity generator for the chassis to solve power limit
 * the velocity generator has constant jerk
 */
#include "chassis_velocity_generator.h"
#include "math.h"
#include "ch.h"

#define MAX_JERK    1.0f // maximum acceleration dot
static float t_prev;
static float v_setpoint_prev = 0;

// version I simple one, always start from 0, shouldn't work yet
// TODO: initialization
float chassis_velocity_generator(float v_setpoint_in)
{
    uint32_t t_sys_time = chVTGetSystemTimeX();
    float t_curr = ST2US(t_sys_time) / 1000000.0f;
    float v_setpoint;
    if (v_setpoint_prev != v_setpoint_in) {
        v_setpoint = v_setpoint_in;
        t_prev = t_curr;
    } else {
        v_setpoint = v_setpoint_prev;
    }

    float k = MAX_JERK;
    float T = (float)sqrt(4.0f * v_setpoint / MAX_JERK);
    float dt = t_curr - t_prev;
    float v_smooth = 0;
    if (dt < 0) {
        v_smooth = v_setpoint;
        // throw error;
    }
    else if (dt < T / 2) {
        v_smooth = 0.5f * k * dt * dt;
    }
    else if (dt < T) {
        v_smooth = v_setpoint - 0.5f * k * (T - dt) * (T - dt);
    }
    else {
        v_smooth = v_setpoint;
    }

    v_setpoint_prev = v_setpoint_in;
    return v_smooth;
}

