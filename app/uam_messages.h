#pragma once
#include <stdint.h>

typedef struct {
    uint64_t t_ms;
    float ax, ay, az;
    float gx, gy, gz;
    float airspeed_mps;
    float alt_m;
} sensor_sample_t;

typedef struct {
    uint64_t t_ms;
    float roll, pitch, yaw;
    float p, q, r;
    float vx, vy, vz;
    float alt_m;
} nav_state_t;

typedef struct {
    uint64_t t_ms;
    float thrust;
    float roll_cmd;
    float pitch_cmd;
    float yaw_cmd;
} control_cmd_t;
