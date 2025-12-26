#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// -------------------------------------------------------------------------
// CONFIGURATION
// -------------------------------------------------------------------------

// RX = D7 (GPIO 20), TX = D6 (GPIO 21) on XIAO ESP32C3
#define RX_PIN 20
#define TX_PIN 21

// Packet Structure (Must match STM32 and Receiver)
typedef struct __attribute__((packed)) {
    uint8_t start_byte; // 0xAA
    
    // Raw Sensor Data (Inputs)
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
    float mag_x,   mag_y,   mag_z;
    
    // AHRS Fused Data (Outputs)
    float roll, pitch, yaw;
    float heading_legacy;
    
    uint8_t end_byte;   // 0x55
} usb_packet_t;

usb_packet_t data;

// Broadcast Address (sends to everyone)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

// -------------------------------------------------------------------------
// CALLBACKS
// -------------------------------------------------------------------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Optional: Blink LED or print status if debugging
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent" : "Fail");
}

// -------------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------------
void setup() {
    // 1. Initialize USB CDC (Virtual Serial)
    Serial.begin(115200);

    // 2. Wait for USB (Safety delay for CDC)
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000)); 

    Serial.println("XIAO ESP32C3 AHRS Bridge (ESP-NOW) Starting...");
    Serial.flush();
    delay(100);

    // 3. Initialize WiFi in Station Mode (Required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    
    // 4. Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // 5. Register Send Callback
    esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);


    // 6. Register Peer (Broadcast)
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }

    // 7. Initialize Hardware UART (Serial1)
    // Warning: D6/D7 are UART0 pins. Ensure "USB CDC On Boot" is Enabled.
    if (Serial) Serial.setDebugOutput(false); // Stop debug logs interfering
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    
    Serial.println("Bridge Ready. Forwarding UART -> ESP-NOW");
}

// -------------------------------------------------------------------------
// LOOP
// -------------------------------------------------------------------------
void loop() {
    // 1. Read from UART (STM32)
    if (Serial1.available() > sizeof(usb_packet_t)) {
        
        // Peek for start byte to stay in sync
        if (Serial1.peek() == 0xAA) {
            
            size_t bytesRead = Serial1.readBytes((uint8_t*)&data, sizeof(usb_packet_t));
            
            if (bytesRead == sizeof(usb_packet_t)) {
                // Validate Packet
                if (data.end_byte == 0x55) {
                    
                    // 2. Send via ESP-NOW (Broadcast)
                    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(usb_packet_t));
                    
                    // Optional: Print to USB for local debugging
                    if (result == ESP_OK) {
                        Serial.printf("Forwarded: R:%.1f P:%.1f Y:%.1f\n", data.roll, data.pitch, data.yaw);
                    } else {
                        Serial.println("ESP-NOW Send Error");
                    }
                    
                } else {
                    // Sync Error
                    Serial.println("Err: Bad Tail");
                }
            }
        } else {
            // Trash byte, skip
            Serial1.read();
        }
    }
}