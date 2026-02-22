#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include "MPU6050.h"
MPU6050 mpu;
// Setup Receiver MAC Address: 3c:8a:1f:a0:1e:74
uint8_t receiverAddress[] = { 0x3C, 0x8A, 0x1F, 0xA0, 0x1E, 0x74 };
// Data structure
typedef struct struct_message {
  int16_t x;   // -1000 to 1000
  int16_t y;  // -1000 to 1000
} struct_message;
struct_message myData;
// Callback when data is sent
void onSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.print("Sent Successfully | X: ");
    Serial.print(myData.x);
    Serial.print("  Y: ");
    Serial.println(myData.y);
  } else {
    Serial.println("Send Failed");
  }
}

void setup() {
  Serial.begin(115200);
  // Initialize I2C for MPU6050
  Wire.begin(21, 22);   // SDA, SCL
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 Connected");
  // ESP-NOW Setup
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }
  esp_now_register_send_cb(onSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1);
  }
  Serial.println("ESP-NOW Ready");
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  // Convert acceleration to tilt angles
  float angleX = atan2(ay, az) * 180 / PI;  // Left / Right
  float angleY = atan2(ax, az) * 180 / PI;  // Forward / Back
  // Limit usable tilt range
  angleX = constrain(angleX, -45, 45);
  angleY = constrain(angleY, -45, 45);
  // Map angle to -1000 to +1000 (same as joystick)
  myData.x = map(angleX, -45, 45, -1000, 1000);
  myData.y = map(angleY, -45, 45, -1000, 1000);
  // Dead zone to prevent shaking
  if (abs(myData.x) < 200) myData.x = 0;
  if (abs(myData.y) < 200) myData.y = 0;
  // Send data
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&myData, sizeof(myData));
  if (result != ESP_OK) {Serial.println("Send Error");}
  delay(50);  // ~20Hz update rate
}