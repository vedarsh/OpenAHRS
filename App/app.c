#include <stdlib.h>

#include "app.h"
#include "math.h"
#include "ahrs.h"
#include "stm32f411xe.h"

volatile bool read_mag_flag = false;
volatile bool read_imu_flag = false;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;

sensor_state_t lis2mdl_state = SENSOR_NOT_INITIALISED;
sensor_state_t icm45686_state = SENSOR_NOT_INITIALISED;

mag_data_out_t mag_data;
imu_data_out_t imu_data;
static usb_packet_t packet;

icm45686_raw_data_t raw;
icm45686_data_t scaled;

float heading;
euler_t orientation;

/* ================= SPI BUS ================= */

const spi_bus_t mag_bus = {
    .hspi = &hspi1,
    .nss_pin = MAG_NSS_Pin,
    .nss_port = MAG_NSS_GPIO_Port
};

const spi_bus_t imu_bus = {
    .hspi = &hspi2,
    .nss_pin = IMU_NSS_Pin,
    .nss_port = IMU_NSS_GPIO_Port
};

/* ================= SENSOR CONFIG ================= */

mag_config_setup_t lis2mdl_setup = {
    .spi_4_mode = true,
    .bdu = true,
    .disable_i2c = true,
    .enable_compensation = true,
    .odr_status = LIS2MDL_ODR_100HZ,
    .mode_status = LIS2MDL_MODE_CONTINUOUS
};

imu_config_setup_t icm45686_setup = {
    .accel_fsr = ICM45686_FSR_8G,
    .gyro_fsr  = ICM45686_FSR_2000DPS,
    .odr       = ICM45686_ODR_1KHZ
};

//calibration data

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
/* ================= MAG INIT ================= */

void lis2mdl_init(const spi_bus_t *bus, sensor_state_t *state)
{
    lis2mdl_set_spi_mode(bus, lis2mdl_setup.spi_4_mode);

    if (lis2mdl_read_chip_id(bus) != 0) {
        *state = SENSOR_ERROR;
        return;
    }

    lis2mdl_soft_reset(bus);
    lis2mdl_set_spi_mode(bus, lis2mdl_setup.spi_4_mode);
    lis2mdl_config_interface(bus, true, lis2mdl_setup.disable_i2c);
    lis2mdl_set_odr(bus, lis2mdl_setup.odr_status);
    lis2mdl_enable_compensation(bus, lis2mdl_setup.enable_compensation);
    lis2mdl_set_mode(bus, LIS2MDL_MODE_CONTINUOUS);

    *state = SENSOR_OK;
}

typedef struct {
    uint8_t addresses[10]; // Max 10 devices
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
                results.addresses[results.count++] = (i << 1); // Store 8-bit address
            }
        }
    }
    return results;
}

/* ================= IMU INIT ================= */

void icm45686_init(const spi_bus_t *bus, sensor_state_t *state)
{
    if (icm45686_read_chip_id(bus) != ICM45686_CHIP_ID) {
        *state = SENSOR_ERROR;
        return;
    }

    icm45686_soft_reset(bus);
    icm45686_configure(bus,
                       icm45686_setup.accel_fsr,
                       icm45686_setup.gyro_fsr,
                       icm45686_setup.odr);

    *state = SENSOR_OK;
}

/* ================= MAG READ (OFFSET + SCALE ONLY) ================= */

static void lis2mdl_read_data(const spi_bus_t *bus,
                              sensor_state_t *state,
                              mag_data_out_t *data)
{
    lis2mdl_data_t raw;
    float temp;

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

    if (!lis2mdl_read_sensor_ready(bus)) {
        data->is_data_stale = true;
        return;
    }

    if (lis2mdl_read_mag(bus, &raw) != 0 ||
        lis2mdl_read_temperature(bus, &temp) != 0) {
        *state = SENSOR_DEGRADED;
        data->is_data_stale = true;
        return;
    }

    data->mag_data_ut.x_ut = (raw.x_ut - mag_offset[0]) * mag_scale[0];
    data->mag_data_ut.y_ut = (raw.y_ut - mag_offset[1]) * mag_scale[1];
    data->mag_data_ut.z_ut = (raw.z_ut - mag_offset[2]) * mag_scale[2];

    data->temp_c = temp;
    data->is_data_stale = false;
    *state = SENSOR_OK;
}

/* ================= IMU READ (BIAS ONLY) ================= */

static void icm45686_read_data(const spi_bus_t *bus,
                               sensor_state_t *state,
                               imu_data_out_t *data)
{

    if (!icm45686_read_sensor_ready(bus)) {
        data->is_data_stale = true;
        return;
    }

    if (icm45686_read_raw_data(bus, &raw) != 0) {
        *state = SENSOR_DEGRADED;
        data->is_data_stale = true;
        return;
    }

    icm45686_convert_data(&raw, &scaled);

    data->imu_data.accel_x_g = scaled.accel_x_g - accel_bias[0];
    data->imu_data.accel_y_g = scaled.accel_y_g - accel_bias[1];
    data->imu_data.accel_z_g = scaled.accel_z_g - accel_bias[2];

    data->imu_data.gyro_x_dps = scaled.gyro_x_dps - gyro_bias[0];
    data->imu_data.gyro_y_dps = scaled.gyro_y_dps - gyro_bias[1];
    data->imu_data.gyro_z_dps = scaled.gyro_z_dps - gyro_bias[2];

    data->imu_data.temp_c = scaled.temp_c;
    data->is_data_stale = false;
    *state = SENSOR_OK;
}

/* ================= HEADING ================= */

static inline void calculate_mag_north(mag_data_out_t *data, float *heading)
{
    float rad = atan2f(data->mag_data_ut.y_ut,
                       data->mag_data_ut.x_ut);
    float deg = rad * (180.0f / M_PI);
    if (deg < 0.0f) deg += 360.0f;
    *heading = deg;
}

/* ================= APP ================= */

bool app_init(void)
{
    lis2mdl_init(&mag_bus, &lis2mdl_state);
    icm45686_init(&imu_bus, &icm45686_state);
    ahrs_init();

    return (lis2mdl_state == SENSOR_OK &&
            icm45686_state == SENSOR_OK);
}

bool app_run(void)
{
    if (read_mag_flag) {
        read_mag_flag = false;
        lis2mdl_read_data(&mag_bus, &lis2mdl_state, &mag_data);
        if (!mag_data.is_data_stale)
            calculate_mag_north(&mag_data, &heading);
    }

    if (read_imu_flag) {
        read_imu_flag = false;
        icm45686_read_data(&imu_bus, &icm45686_state, &imu_data);

        if (!imu_data.is_data_stale) {
            ahrs_update(&imu_data, &mag_data, 0.005f);
            ahrs_get_euler(&orientation);

            packet.start_byte = 0xAA;

            packet.accel_x = imu_data.imu_data.accel_x_g;
            packet.accel_y = imu_data.imu_data.accel_y_g;
            packet.accel_z = imu_data.imu_data.accel_z_g;

            packet.gyro_x  = imu_data.imu_data.gyro_x_dps;
            packet.gyro_y  = imu_data.imu_data.gyro_y_dps;
            packet.gyro_z  = imu_data.imu_data.gyro_z_dps;

            packet.mag_x   = mag_data.mag_data_ut.x_ut;
            packet.mag_y   = mag_data.mag_data_ut.y_ut;
            packet.mag_z   = mag_data.mag_data_ut.z_ut;

            packet.roll  = orientation.roll;
            packet.pitch = orientation.pitch;
            packet.yaw   = orientation.yaw;
            packet.heading_legacy = heading;

            packet.end_byte = 0x55;
        }
    }

    check_dev_on_i2c(&hi2c1);
    return true;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM10) read_imu_flag = true;
    if (htim->Instance == TIM11) read_mag_flag = true;
}
