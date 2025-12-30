#include <stdlib.h>

#include "app.h"
#include "math.h"
#include "ahrs.h"
#include "tlm.h"
#include "sensor_types.h"
#include "stm32f411xe.h"

#include "imu.h"
#include "mag.h"

volatile bool read_mag_flag = false;
volatile bool read_imu_flag = false;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

// Sensor context instances
mag_ctx_t *mag_ctx;
imu_ctx_t *imu_ctx;

// Application data structures
mag_data_out_t mag_data;
imu_data_out_t imu_data;
usb_packet_t packet;

float heading;
euler_t orientation;

// TLM counter
uint32_t ahrs_update_count = 0;

/* ================= CALIBRATION DATA ================= */

static const float accel_bias[3] = {
    0.0184530843f,
    -0.0007122396f,
    0.00374943f
};

static const float gyro_bias[3] = {
    -0.35056911f,
    0.09686992f,
    0.33424798f
};

static const float mag_offset[3] = {
    13.10572529f,
    -7.50889969f,
    -10.97933769f
};

static const float mag_scale[3] = {
    0.96661077f,
    0.98443416f,
    1.04895507f
};

/* ================= HELPER FUNCTIONS ================= */

typedef struct {
    uint8_t addresses[10];
    uint8_t count;
} i2c_scan_t;

i2c_scan_t check_dev_on_i2c(I2C_HandleTypeDef *hi2c)
{
    i2c_scan_t results = {0};

    for (uint16_t i = 0; i < 128; i++)
    {
        if (HAL_I2C_IsDeviceReady(hi2c, (i << 1), 2, 5) == HAL_OK)
        {
            if (results.count < 10) {
                results.addresses[results.count++] = (i << 1);
            }
        }
    }
    return results;
}

/* ================= MAG READ WITH CALIBRATION (FIXED) ================= */

static void mag_read_calibrated(mag_ctx_t *ctx, mag_data_out_t *data)
{
    mag_output_t raw_mag;

    // FIXED: In continuous mode, don't check data ready, just read [web:51][web:56]
    // The sensor updates at its configured ODR automatically
    if (mag_read_data(ctx, &raw_mag) != SENSOR_OK) {
        data->is_data_stale = true;
        return;
    }

    // Apply hard-iron offset and soft-iron scale correction
    data->mag_data_ut.x_ut = (raw_mag.mag_x_ut - mag_offset[0]) * mag_scale[0];
    data->mag_data_ut.y_ut = (raw_mag.mag_y_ut - mag_offset[1]) * mag_scale[1];
    data->mag_data_ut.z_ut = (raw_mag.mag_z_ut - mag_offset[2]) * mag_scale[2];

    data->temp_c = raw_mag.temp_c;
    data->is_data_stale = false;
}

/* ================= IMU READ WITH CALIBRATION ================= */

static void imu_read_calibrated(imu_ctx_t *ctx, imu_data_out_t *data)
{
    imu_output_t raw_imu;

    // Read raw IMU data (data ready check is inside if needed)
    if (imu_read_data(ctx, &raw_imu) != SENSOR_OK) {
        data->is_data_stale = true;
        return;
    }

    // Apply bias correction
    data->imu_data.accel_x_g = raw_imu.acc_x_g - accel_bias[0];
    data->imu_data.accel_y_g = raw_imu.acc_y_g - accel_bias[1];
    data->imu_data.accel_z_g = raw_imu.acc_z_g - accel_bias[2];

    data->imu_data.gyro_x_dps = raw_imu.gyr_x_dps - gyro_bias[0];
    data->imu_data.gyro_y_dps = raw_imu.gyr_y_dps - gyro_bias[1];
    data->imu_data.gyro_z_dps = raw_imu.gyr_z_dps - gyro_bias[2];

    data->imu_data.temp_c = raw_imu.temp_c;
    data->is_data_stale = false;
}

/* ================= HEADING CALCULATION ================= */

static inline void calculate_mag_north(mag_data_out_t *data, float *heading)
{
    float rad = atan2f(data->mag_data_ut.y_ut,
                       data->mag_data_ut.x_ut);
    float deg = rad * (180.0f / M_PI);
    if (deg < 0.0f) deg += 360.0f;
    *heading = deg;
}

/* ================= APP INITIALIZATION ================= */

bool app_init(void)
{
    // Create sensor contexts using wrapper API
    mag_ctx = create_mag_ctx(&hspi1, MAG_NSS_GPIO_Port, MAG_NSS_Pin);
    imu_ctx = create_imu_ctx(&hspi2, IMU_NSS_GPIO_Port, IMU_NSS_Pin);

    // Configure magnetometer (continuous mode is critical) [web:51]
    mag_ctx->mag_config.odr = LIS2MDL_ODR_100HZ;
    mag_ctx->mag_config.mode = LIS2MDL_MODE_CONTINUOUS;  // Critical for continuous readings
    mag_ctx->mag_config.temp_comp_enabled = true;

    // Configure IMU
    imu_ctx->imu_config.accel_fsr = ICM45686_FSR_8G;
    imu_ctx->imu_config.gyro_fsr = ICM45686_FSR_2000DPS;
    imu_ctx->imu_config.odr = ICM45686_ODR_1KHZ;

    // Initialize sensors
    sensor_state_t mag_state = init_mag(mag_ctx);
    sensor_state_t imu_state = init_imu(imu_ctx);

    // Give magnetometer time to settle after init [web:51]
    HAL_Delay(100);

    // Initialize AHRS and telemetry
    ahrs_init();
    tlm_init();

    return (mag_state == SENSOR_OK && imu_state == SENSOR_OK);
}

/* ================= APP MAIN LOOP ================= */

bool app_run(void)
{
    // Magnetometer update at 100Hz
    if (read_mag_flag) {
        read_mag_flag = false;
        mag_read_calibrated(mag_ctx, &mag_data);
        
        if (!mag_data.is_data_stale) {
            calculate_mag_north(&mag_data, &heading);
        }
    }

    // IMU update at 200Hz (sensor fusion rate)
    if (read_imu_flag) {
        read_imu_flag = false;
        imu_read_calibrated(imu_ctx, &imu_data);

        if (!imu_data.is_data_stale) {
            // Sensor fusion with 5ms timestep
            ahrs_update(&imu_data, &mag_data, 0.005f);
            ahrs_get_euler(&orientation);
            
            // Increment counter for telemetry
            ahrs_update_count++;

            // Prepare telemetry packet (legacy format)
            packet.start_byte = 0xAA;

            packet.accel_x = imu_data.imu_data.accel_x_g;
            packet.accel_y = imu_data.imu_data.accel_y_g;
            packet.accel_z = imu_data.imu_data.accel_z_g;

            packet.gyro_x = imu_data.imu_data.gyro_x_dps;
            packet.gyro_y = imu_data.imu_data.gyro_y_dps;
            packet.gyro_z = imu_data.imu_data.gyro_z_dps;

            packet.mag_x = mag_data.mag_data_ut.x_ut;
            packet.mag_y = mag_data.mag_data_ut.y_ut;
            packet.mag_z = mag_data.mag_data_ut.z_ut;

            packet.roll = orientation.roll;
            packet.pitch = orientation.pitch;
            packet.yaw = orientation.yaw;
            packet.heading_legacy = heading;

            packet.end_byte = 0x55;
        }
    }

    // Process telemetry commands
    process_tlm();

    return true;
}

/* ================= TIMER CALLBACK ================= */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM10) read_imu_flag = true;   // 200Hz
    if (htim->Instance == TIM11) read_mag_flag = true;   // 100Hz
}
