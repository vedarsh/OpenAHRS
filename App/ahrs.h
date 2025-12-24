#ifndef AHRS_H
#define AHRS_H

#include <math.h>
#include <stdint.h>
#include "app.h"

// Tuning Parameter: Proportional Gain
// 0.1 is standard. Lower (0.05) = smoother but slower. Higher (0.5) = responsive but noisy.
#define AHRS_BETA 0.1f 

typedef struct {
    float q0, q1, q2, q3;
} quaternion_t;

typedef struct {
    float roll, pitch, yaw;
} euler_t;

// Initialize quaternion to identity
void ahrs_init(void);

// Main update function
// dt_sec: Delta time in seconds since last call (e.g., 0.001f for 1kHz)
void ahrs_update(const imu_data_out_t *imu, const mag_data_out_t *mag, float dt_sec);

// Convert internal quaternion to readable angles
void ahrs_get_euler(euler_t *out);

#endif
