#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// -------------------------------------------------------------------------
// CONFIGURATION
// -------------------------------------------------------------------------

// Packet Structure (54 Bytes)
typedef struct __attribute__((packed)) {
    uint8_t start_byte; 
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
    float mag_x,   mag_y,   mag_z;
    float roll, pitch, yaw;
    float heading_legacy;
    uint8_t end_byte;   
} usb_packet_t;

usb_packet_t receivedData;
volatile int8_t g_last_rssi = 0; // Store RSSI here

// -------------------------------------------------------------------------
// PROMISCUOUS CALLBACK (To get RSSI)
// -------------------------------------------------------------------------
void promiscuous_rx_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
    // We sniff the packet metadata to get the signal strength
    wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
    if (type == WIFI_PKT_DATA) {
        g_last_rssi = pkt->rx_ctrl.rssi;
    }
}

// -------------------------------------------------------------------------
// ESP-NOW CALLBACK
// -------------------------------------------------------------------------
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    // 1. Verify Packet Size
    if (len == sizeof(usb_packet_t)) {
        memcpy(&receivedData, incomingData, sizeof(receivedData));

        if (receivedData.start_byte == 0xAA && receivedData.end_byte == 0x55) {
            
            // 2. Send Packet (54 bytes)
            Serial.write((uint8_t*)&receivedData, sizeof(usb_packet_t));
            
            // 3. Append RSSI (1 byte)
            // Python must expect 55 bytes total now
            Serial.write((uint8_t)g_last_rssi);
        }
    }
}

// -------------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10); 

    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    WiFi.disconnect(); 

    // Enable Promiscuous Mode for RSSI
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);

    if (esp_now_init() != ESP_OK) {
        return;
    }

    esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
}

void loop() {
    delay(1000); 
}
