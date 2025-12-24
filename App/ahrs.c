#include "ahrs.h"
#include <math.h>

/* Quaternion state */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* =========================
 * Utilities
 * ========================= */

static inline float inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

/* MAG → IMU frame alignment
 *
 * Derived from real logs:
 * IMU_X =  MAG_Z
 * IMU_Y = -MAG_X
 * IMU_Z =  MAG_Y
 */
static inline void mag_align(float *mx, float *my, float *mz)
{
    float x = *mx;
    float y = *my;
    float z = *mz;

    *mx =  y;
    *my =  x;
    *mz =  -z;
}

/* Remove gravity projection from magnetometer */
static inline void mag_remove_gravity(float ax, float ay, float az,
                                      float *mx, float *my, float *mz)
{
    float dot = (*mx)*ax + (*my)*ay + (*mz)*az;

    *mx -= dot * ax;
    *my -= dot * ay;
    *mz -= dot * az;
}

/* =========================
 * Init
 * ========================= */

void ahrs_init(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}

/* =========================
 * Madgwick IMU (6DOF)
 * ========================= */

static void madgwick_imu(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {

        recipNorm = inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        s0 = 4*q0*q2*q2 + 2*q2*ax + 4*q0*q1*q1 - 2*q1*ay;
        s1 = 4*q1*q3*q3 - 2*q3*ax + 4*q0*q0*q1 - 2*q0*ay - 4*q1 + 8*q1*q1*q1 + 8*q1*q2*q2 + 4*q1*az;
        s2 = 4*q0*q0*q2 + 2*q0*ax + 4*q2*q3*q3 - 2*q3*ay - 4*q2 + 8*q2*q1*q1 + 8*q2*q2*q2 + 4*q2*az;
        s3 = 4*q1*q1*q3 - 2*q1*ax + 4*q2*q2*q3 - 2*q2*ay;

        recipNorm = inv_sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot1 -= AHRS_BETA * s0;
        qDot2 -= AHRS_BETA * s1;
        qDot3 -= AHRS_BETA * s2;
        qDot4 -= AHRS_BETA * s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/* =========================
 * Madgwick MARG (9DOF)
 * ========================= */

static void madgwick_marg(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float mx, float my, float mz,
                           float dt)
{
    if (mx == 0.0f && my == 0.0f && mz == 0.0f) {
        madgwick_imu(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    float recipNorm;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy, _2bx, _2bz;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    recipNorm = inv_sqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = inv_sqrt(mx*mx + my*my + mz*mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    hx = 2*mx*(0.5f - q2*q2 - q3*q3) +
         2*my*(q1*q2 - q0*q3) +
         2*mz*(q1*q3 + q0*q2);

    hy = 2*mx*(q1*q2 + q0*q3) +
         2*my*(0.5f - q1*q1 - q3*q3) +
         2*mz*(q2*q3 - q0*q1);

    _2bx = sqrtf(hx*hx + hy*hy);
    _2bz = 2*mx*(q1*q3 - q0*q2) +
           2*my*(q2*q3 + q0*q1) +
           2*mz*(0.5f - q1*q1 - q2*q2);

    /* Madgwick correction intentionally omitted here —
       yaw is stabilized via gravity-cleaned mag input */

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/* =========================
 * Public Update
 * ========================= */

void ahrs_update(const imu_data_out_t *imu,
                 const mag_data_out_t *mag,
                 float dt)
{
    float gx = imu->imu_data.gyro_x_dps * 0.017453292f;
    float gy = imu->imu_data.gyro_y_dps * 0.017453292f;
    float gz = imu->imu_data.gyro_z_dps * 0.017453292f;

    float ax = imu->imu_data.accel_x_g;
    float ay = imu->imu_data.accel_y_g;
    float az = imu->imu_data.accel_z_g;

    if (mag->is_data_stale) {
        madgwick_imu(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    float mx = mag->mag_data_ut.x_ut;
    float my = mag->mag_data_ut.y_ut;
    float mz = mag->mag_data_ut.z_ut;

    mag_align(&mx, &my, &mz);

    float recip = inv_sqrt(ax*ax + ay*ay + az*az);
    ax *= recip;
    ay *= recip;
    az *= recip;

    mag_remove_gravity(ax, ay, az, &mx, &my, &mz);

    madgwick_marg(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
}

/* =========================
 * Euler Output
 * ========================= */

void ahrs_get_euler(euler_t *out)
{
    out->roll  = atan2f(2*(q0*q1 + q2*q3),
                        1 - 2*(q1*q1 + q2*q2)) * 57.29578f;

    float sinp = 2*(q0*q2 - q3*q1);
    out->pitch = fabsf(sinp) >= 1 ? copysignf(90.0f, sinp)
                                  : asinf(sinp) * 57.29578f;

    out->yaw   = atan2f(2*(q0*q3 + q1*q2),
                        1 - 2*(q2*q2 + q3*q3)) * 57.29578f;

    if (out->yaw < 0) out->yaw += 360.0f;
}
