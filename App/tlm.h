#ifndef TLM_H
#define TLM_H

#include "stm32f4xx.h"

#include "main.h"

#define GET_TLM_VERB   0x01
#define GET_TLM_FUSED  0x02
#define GET_TLM_SENS   0x03

#define PING            0x10
#define GET_NTP_TIME_CMD 0x11


typedef struct __attribute__((packed)) {
    
    uint8_t start_byte;
    
    float accel_x;
    float accel_y;
    float accel_z;
    
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    float mag_x;
    float mag_y;
    float mag_z;

    float roll;
    float pitch;
    float yaw;
    float heading_legacy;
    
    uint8_t end_byte;   // 0x55
} usb_packet_t;

#endif
