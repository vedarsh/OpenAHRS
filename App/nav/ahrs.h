#ifndef AHRS_H
#define AHRS_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "app.h"

/* ================= TUNING PARAMETERS ================= */

// Accelerometer correction gains
#define KP_ACC   2.5f   // Proportional gain
#define KI_ACC   0.05f  // Integral gain (bias correction)

// Magnetometer correction gains (yaw only, very weak)
#define KP_MAG   0.15f
#define KI_MAG   0.002f

// Gyro magnitude threshold to gate magnetometer (rad/s)
#define GYRO_MAG_GATE_RAD   0.2f   // ~11.5 deg/s

/* ================= DATA STRUCTURES ================= */

typedef struct {
    float q0, q1, q2, q3;  // w, x, y, z
} quaternion_t;

typedef struct {
    float roll, pitch, yaw;  // degrees
} euler_t;

typedef struct {
    float x, y, z;  // rad/s
} gyro_bias_t;

/* ================= PUBLIC API ================= */

/**
 * @brief Initialize AHRS filter to identity quaternion
 */
void ahrs_init(void);

/**
 * @brief Reset AHRS filter to identity quaternion
 * @note Use when aircraft orientation is lost or needs re-initialization
 */
void ahrs_reset(void);

/**
 * @brief Update AHRS filter with new sensor data
 * @param imu Calibrated IMU data (accel in g, gyro in deg/s)
 * @param mag Calibrated magnetometer data (in uT)
 * @param dt_sec Time step in seconds (e.g., 0.005f for 200Hz)
 */
void ahrs_update(const imu_data_out_t *imu, const mag_data_out_t *mag, float dt_sec);

/**
 * @brief Get current orientation as Euler angles
 * @param out Pointer to euler_t structure to fill
 * @note Angles in degrees: roll/pitch [-180,180], yaw [0,360]
 */
void ahrs_get_euler(euler_t *out);

/**
 * @brief Get current orientation quaternion
 * @param out Pointer to quaternion_t structure to fill
 * @note Quaternion format: q0=w (scalar), q1=x, q2=y, q3=z
 */
void ahrs_get_quaternion(quaternion_t *out);

/**
 * @brief Get estimated gyroscope bias
 * @param out Pointer to gyro_bias_t structure to fill
 * @note Bias in rad/s
 */
void ahrs_get_gyro_bias(gyro_bias_t *out);

/**
 * @brief Check if AHRS filter has converged
 * @return true if filter is stable, false otherwise
 * @note Useful for determining when to trust orientation output
 */
bool ahrs_is_converged(void);

#endif // AHRS_H
