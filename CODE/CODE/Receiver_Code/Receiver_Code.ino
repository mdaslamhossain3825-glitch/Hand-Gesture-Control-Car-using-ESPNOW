#include <esp_now.h>
#include <WiFi.h>
#define ENA 14
#define ENB 32
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define SIGNAL_TIMEOUT 500  // 500ms failsafe
unsigned long lastReceiveTime = 0;
unsigned char deadzone = 300;
typedef struct struct_message {
  int16_t x;
  int16_t y;
} struct_message;
struct_message incomingData;
/////////////////RECEIVE CALLBACK////////////////////////
void onReceive(const uint8_t *mac, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  lastReceiveTime = millis();  // Update signal timer
  Serial.printf("Tilt X: %d  Y: %d\n", incomingData.x, incomingData.y);
  driveMotors(incomingData.x, incomingData.y);
}
/////////////////MOTOR DRIVE LOGIC///////////////////
void driveMotors(int16_t x, int16_t y) {
  if (abs(x) < deadzone) x = 0;
  if (abs(y) < deadzone) y = 0;
  // Tank mixing
  int16_t motorA = y + x;
  int16_t motorB = y - x;
  motorA = constrain(motorA, -1000, 1000);
  motorB = constrain(motorB, -1000, 1000);
  // Map -1000..1000 → -255..255
  motorA = map(motorA, -1000, 1000, -255, 255);
  motorB = map(motorB, -1000, 1000, -255, 255);
  setMotor(ENA, IN1, IN2, motorA);
  setMotor(ENB, IN3, IN4, motorB);
}
////////////////////SET MOTOR/////////////////////
void setMotor(int pwmPin, int dirPin1, int dirPin2, int speed) {
  if (speed > 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    analogWrite(pwmPin, 0);
  }
}
/////////////////STOP ALL MOTORS///////////////
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1)
      ;
  }
  esp_now_register_recv_cb(onReceive);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stopMotors();
}
//////////////////LOOP///////////////////
void loop() {
  // Failsafe: stop if signal lost
  if (millis() - lastReceiveTime > SIGNAL_TIMEOUT) {
    stopMotors();
  }
}