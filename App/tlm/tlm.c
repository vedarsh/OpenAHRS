#include "tlm.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "app.h"
#include "mag.h"
#include "imu.h"
#include "ahrs.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart1;

// External sensor contexts
extern mag_ctx_t *mag_ctx;
extern imu_ctx_t *imu_ctx;

// External data
extern mag_data_out_t mag_data;
extern imu_data_out_t imu_data;
extern euler_t orientation;
extern float heading;
extern usb_packet_t packet;

/* ================= TELEMETRY STATE ================= */

volatile uint8_t rx_buf[2];
volatile uint8_t tlm_cmd_noun = 0;
volatile uint8_t tlm_cmd_verb = 0;
volatile bool is_tlm_received = false;

static uint32_t error_count = 0;
static uint32_t ahrs_update_count = 0;
static uint32_t uptime_ms = 0;

/* ================= CHECKSUM CALCULATION ================= */

static uint8_t calculate_checksum(const uint8_t *data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum ^= data[i];
    }
    return sum;
}

/* ================= PACKET BUILDERS ================= */

static void build_fused_packet(tlm_fused_t *pkt)
{
    pkt->start_byte = 0xF1;
    pkt->roll = orientation.roll;
    pkt->pitch = orientation.pitch;
    pkt->yaw = orientation.yaw;
    pkt->heading = heading;
    
    // Get quaternion from AHRS (implement ahrs_get_quaternion)
    // For now, set to identity quaternion
    pkt->qw = 1.0f;
    pkt->qx = 0.0f;
    pkt->qy = 0.0f;
    pkt->qz = 0.0f;
    
    pkt->checksum = calculate_checksum((uint8_t*)pkt, offsetof(tlm_fused_t, checksum));
    pkt->end_byte = 0x1F;
}

static void build_sensor_packet(tlm_sensor_t *pkt)
{
    pkt->start_byte = 0xF2;
    pkt->accel_x_g = imu_data.imu_data.accel_x_g;
    pkt->accel_y_g = imu_data.imu_data.accel_y_g;
    pkt->accel_z_g = imu_data.imu_data.accel_z_g;
    pkt->gyro_x_dps = imu_data.imu_data.gyro_x_dps;
    pkt->gyro_y_dps = imu_data.imu_data.gyro_y_dps;
    pkt->gyro_z_dps = imu_data.imu_data.gyro_z_dps;
    pkt->mag_x_ut = mag_data.mag_data_ut.x_ut;
    pkt->mag_y_ut = mag_data.mag_data_ut.y_ut;
    pkt->mag_z_ut = mag_data.mag_data_ut.z_ut;
    pkt->imu_temp_c = imu_data.imu_data.temp_c;
    pkt->mag_temp_c = mag_data.temp_c;
    
    pkt->checksum = calculate_checksum((uint8_t*)pkt, offsetof(tlm_sensor_t, checksum));
    pkt->end_byte = 0x2F;
}

static void build_health_packet(tlm_health_t *pkt)
{
    pkt->start_byte = 0xF4;
    pkt->imu_health = imu_ctx->health;
    pkt->mag_health = mag_ctx->health;
    pkt->imu_error_count = 0; // Track these in your app
    pkt->mag_error_count = 0;
    pkt->ahrs_update_count = ahrs_update_count;
    pkt->uptime_ms = HAL_GetTick();
    pkt->cpu_usage_percent = 0; // Implement if needed
    
    pkt->checksum = calculate_checksum((uint8_t*)pkt, offsetof(tlm_health_t, checksum));
    pkt->end_byte = 0x4F;
}

static void build_sys_params_packet(tlm_sys_params_t *pkt)
{
    pkt->start_byte = 0xF5;
    pkt->imu_accel_fsr = imu_ctx->imu_config.accel_fsr;
    pkt->imu_gyro_fsr = imu_ctx->imu_config.gyro_fsr;
    pkt->imu_odr = imu_ctx->imu_config.odr;
    pkt->mag_odr = mag_ctx->mag_config.odr;
    pkt->mag_mode = mag_ctx->mag_config.mode;
    pkt->mag_temp_comp = mag_ctx->mag_config.temp_comp_enabled;
    pkt->imu_sample_rate_hz = 200; // Your configured rate
    pkt->mag_sample_rate_hz = 100;
    
    pkt->checksum = calculate_checksum((uint8_t*)pkt, offsetof(tlm_sys_params_t, checksum));
    pkt->end_byte = 0x5F;
}

/* ================= COMMAND HANDLERS ================= */

static void handle_get_tlm_fused(void)
{
    tlm_fused_t pkt;
    build_fused_packet(&pkt);
    HAL_UART_Transmit(&huart1, (uint8_t*)&pkt, sizeof(pkt), 100);
}

static void handle_get_tlm_sens(void)
{
    tlm_sensor_t pkt;
    build_sensor_packet(&pkt);
    HAL_UART_Transmit(&huart1, (uint8_t*)&pkt, sizeof(pkt), 100);
}

static void handle_get_sys_health(void)
{
    tlm_health_t pkt;
    build_health_packet(&pkt);
    HAL_UART_Transmit(&huart1, (uint8_t*)&pkt, sizeof(pkt), 100);
}

static void handle_get_sys_params(void)
{
    tlm_sys_params_t pkt;
    build_sys_params_packet(&pkt);
    HAL_UART_Transmit(&huart1, (uint8_t*)&pkt, sizeof(pkt), 100);
}

static void handle_set_mag_params(uint8_t verb)
{
    sensor_state_t result = SENSOR_ERROR;
    
    switch (verb) {
        case MAG_SET_ODR:
            // ODR values would come from a third byte (extend protocol)
            mag_ctx->mag_config.odr = LIS2MDL_ODR_100HZ;
            result = mag_reconfigure(mag_ctx);
            break;
            
        case MAG_ENABLE_COMP:
            mag_ctx->mag_config.temp_comp_enabled = true;
            result = mag_reconfigure(mag_ctx);
            break;
            
        case MAG_DISABLE_COMP:
            mag_ctx->mag_config.temp_comp_enabled = false;
            result = mag_reconfigure(mag_ctx);
            break;
            
        default:
            tlm_send_response(RESP_NACK, SET_MAG_PARAMS, 0x01);
            return;
    }
    
    if (result == SENSOR_OK) {
        tlm_send_response(RESP_ACK, SET_MAG_PARAMS, 0x00);
    } else {
        tlm_send_response(RESP_ERROR, SET_MAG_PARAMS, result);
    }
}

static void handle_set_imu_params(uint8_t verb)
{
    sensor_state_t result = SENSOR_ERROR;
    
    switch (verb) {
        case IMU_SET_ACCEL_FSR:
            imu_ctx->imu_config.accel_fsr = ICM45686_FSR_8G;
            result = imu_reconfigure(imu_ctx);
            break;
            
        case IMU_SET_GYRO_FSR:
            imu_ctx->imu_config.gyro_fsr = ICM45686_FSR_2000DPS;
            result = imu_reconfigure(imu_ctx);
            break;
            
        default:
            tlm_send_response(RESP_NACK, SET_IMU_PARAMS, 0x01);
            return;
    }
    
    if (result == SENSOR_OK) {
        tlm_send_response(RESP_ACK, SET_IMU_PARAMS, 0x00);
    } else {
        tlm_send_response(RESP_ERROR, SET_IMU_PARAMS, result);
    }
}

static void handle_ping(void)
{
    tlm_send_response(RESP_ACK, CMD_PING, 0x00);
}

static void handle_reset_comms(void)
{
    error_count = 0;
    tlm_send_response(RESP_ACK, CMD_RESET_COMMS, 0x00);
}

static void handle_soft_reset(void)
{
    sensor_state_t imu_result = imu_reconfigure(imu_ctx);
    sensor_state_t mag_result = mag_reconfigure(mag_ctx);
    
    if (imu_result == SENSOR_OK && mag_result == SENSOR_OK) {
        tlm_send_response(RESP_ACK, CMD_SOFT_RESET, 0x00);
    } else {
        tlm_send_response(RESP_ERROR, CMD_SOFT_RESET, 0xFF);
    }
}

/* ================= COMMAND DISPATCHER ================= */

static void dispatch_command(uint8_t noun, uint8_t verb)
{
    switch (noun) {
        case GET_TLM_FUSED:
            handle_get_tlm_fused();
            break;
            
        case GET_TLM_SENS:
            handle_get_tlm_sens();
            break;
            
        case GET_SYS_HEALTH:
            handle_get_sys_health();
            break;
            
        case GET_SYS_PARAMS:
            handle_get_sys_params();
            break;
            
        case SET_MAG_PARAMS:
            handle_set_mag_params(verb);
            break;
            
        case SET_IMU_PARAMS:
            handle_set_imu_params(verb);
            break;
            
        case CMD_PING:
            handle_ping();
            break;
            
        case CMD_RESET_COMMS:
            handle_reset_comms();
            break;
            
        case CMD_SOFT_RESET:
            handle_soft_reset();
            break;
            
        default:
            error_count++;
            tlm_send_response(RESP_NACK, noun, 0xFF);
            break;
    }
}

/* ================= PUBLIC API IMPLEMENTATION ================= */

void tlm_init(void)
{
    HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buf, sizeof(rx_buf));
}

void process_tlm(void)
{
    if (is_tlm_received) {
        is_tlm_received = false;
        dispatch_command(tlm_cmd_noun, tlm_cmd_verb);
    }
}

void tlm_send_response(uint8_t response_code, uint8_t command, uint8_t status)
{
    tlm_response_t resp;
    resp.start_byte = response_code;
    resp.command_echo = command;
    resp.status_code = status;
    resp.end_byte = 0x55;
    
    HAL_UART_Transmit(&huart1, (uint8_t*)&resp, sizeof(resp), 50);
}

void tlm_send_fused(const tlm_fused_t *packet)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)packet, sizeof(tlm_fused_t), 100);
}

void tlm_send_sensor(const tlm_sensor_t *packet)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)packet, sizeof(tlm_sensor_t), 100);
}

/* ================= UART CALLBACK ================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        tlm_cmd_noun = rx_buf[0];
        tlm_cmd_verb = rx_buf[1];
        is_tlm_received = true;
        
        // Re-enable interrupt for next command
        HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buf, sizeof(rx_buf));
    }
}
