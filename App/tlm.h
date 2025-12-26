#ifndef TLM_H
#define TLM_H

#include "stm32f4xx.h"

#include "main.h"


typedef struct __attribute__((packed)) {
    
    uint8_t start_byte; // 0xAA
    
    // Raw Sensor Data (Inputs)
    float accel_x;
    float accel_y;
    float accel_z;
    
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    float mag_x;
    float mag_y;
    float mag_z;
    
    // AHRS Fused Data (Outputs)
    float roll;
    float pitch;
    float yaw;
    float heading_legacy;
    
    uint8_t end_byte;   // 0x55
} usb_packet_t;

#endif
