#include "ahrs.h"
#include <math.h>

/* =========================================================
 * Quaternion state (body -> navigation)
 * ========================================================= */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* Gyro bias estimate (rad/s) */
static float bgx = 0.0f, bgy = 0.0f, bgz = 0.0f;

/* =========================================================
 * Gains (tuned for BNO055-like behavior)
 * ========================================================= */

/* Accelerometer correction (roll/pitch) */
#define KP_ACC   2.5f
#define KI_ACC   0.05f

/* Magnetometer correction (yaw only, very weak) */
#define KP_MAG   0.15f
#define KI_MAG   0.002f

/* Disable mag correction during fast rotation */
#define GYRO_MAG_GATE_RAD   0.2f   /* ~11.5 deg/s */

/* =========================================================
 * Utilities
 * ========================================================= */

static inline float inv_sqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    return 1.0f / sqrtf(x);
}

/* =========================================================
 * Init
 * ========================================================= */

void ahrs_init(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    bgx = 0.0f;
    bgy = 0.0f;
    bgz = 0.0f;
}

/* =========================================================
 * Core INS-like AHRS update (Mahony-style)
 * ========================================================= */

static void ahrs_core(float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float mx, float my, float mz,
                      int mag_valid,
                      float dt)
{
    /* -------------------------
     * Normalize accelerometer
     * ------------------------- */
    float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an <= 0.0f)
        return;

    ax /= an;
    ay /= an;
    az /= an;

    /* -------------------------
     * Estimated gravity vector
     * (from quaternion)
     * ------------------------- */
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /* -------------------------
     * Accelerometer error
     * (cross product)
     * ------------------------- */
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
//    float ez = (ax * vy - ay * vx); Never used anyways

    /* -------------------------
     * Magnetometer yaw error
     * (ONLY yaw, very weak)
     * ------------------------- */
    float emz = 0.0f;

    float gyro_norm = fabsf(gx) + fabsf(gy) + fabsf(gz);

    if (mag_valid && gyro_norm < GYRO_MAG_GATE_RAD) {

        float mn = sqrtf(mx*mx + my*my + mz*mz);
        if (mn > 0.0f) {
            mx /= mn;
            my /= mn;
            mz /= mn;

            /* Reference direction of Earth's field */
            float hx = 2.0f*mx*(0.5f - q2*q2 - q3*q3)
                     + 2.0f*my*(q1*q2 - q0*q3)
                     + 2.0f*mz*(q1*q3 + q0*q2);

            float hy = 2.0f*mx*(q1*q2 + q0*q3)
                     + 2.0f*my*(0.5f - q1*q1 - q3*q3)
                     + 2.0f*mz*(q2*q3 - q0*q1);

            float bx = sqrtf(hx*hx + hy*hy);

            /* Estimated mag direction (yaw plane only) */
            float wx = 2.0f * bx * (0.5f - q2*q2 - q3*q3);
            float wy = 2.0f * bx * (q1*q2 - q0*q3);

            /* Yaw error */
            emz = (mx * wy - my * wx);
        }
    }

    /* -------------------------
     * Integrate gyro bias
     * ------------------------- */
    bgx += KI_ACC * ex * dt;
    bgy += KI_ACC * ey * dt;
    bgz += KI_MAG * emz * dt;

    /* -------------------------
     * Apply corrections
     * ------------------------- */
    gx -= bgx;
    gy -= bgy;
    gz -= bgz;

    gx += KP_ACC * ex;
    gy += KP_ACC * ey;
    gz += KP_MAG * emz;

    /* -------------------------
     * Quaternion integration
     * ------------------------- */
    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    /* -------------------------
     * Normalize quaternion
     * ------------------------- */
    float recip = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recip;
    q1 *= recip;
    q2 *= recip;
    q3 *= recip;
}

/* =========================================================
 * Public Update (UNCHANGED signature)
 * ========================================================= */

void ahrs_update(const imu_data_out_t *imu,
                 const mag_data_out_t *mag,
                 float dt)
{
    /* Gyro -> rad/s */
    float gx = imu->imu_data.gyro_x_dps * 0.017453292f;
    float gy = imu->imu_data.gyro_y_dps * 0.017453292f;
    float gz = imu->imu_data.gyro_z_dps * 0.017453292f;

    /* Accel (g) */
    float ax = imu->imu_data.accel_x_g;
    float ay = imu->imu_data.accel_y_g;
    float az = imu->imu_data.accel_z_g;

    /* Mag */
    int mag_valid = !mag->is_data_stale;

    float mx = mag->mag_data_ut.x_ut;
    float my = mag->mag_data_ut.y_ut;
    float mz = mag->mag_data_ut.z_ut;

    ahrs_core(gx, gy, gz,
              ax, ay, az,
              mx, my, mz,
              mag_valid,
              dt);
}

/* =========================================================
 * Euler Output (UNCHANGED)
 * ========================================================= */

void ahrs_get_euler(euler_t *out)
{
    out->roll = atan2f(
        2.0f * (q0*q1 + q2*q3),
        1.0f - 2.0f * (q1*q1 + q2*q2)
    ) * 57.29578f;

    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (fabsf(sinp) >= 1.0f)
        out->pitch = copysignf(90.0f, sinp);
    else
        out->pitch = asinf(sinp) * 57.29578f;

    out->yaw = atan2f(
        2.0f * (q0*q3 + q1*q2),
        1.0f - 2.0f * (q2*q2 + q3*q3)
    ) * 57.29578f;

    if (out->yaw < 0.0f)
        out->yaw += 360.0f;
}
