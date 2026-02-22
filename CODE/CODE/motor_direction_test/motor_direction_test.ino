#define ENA 14
#define ENB 32
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define CALIB_SPEED 200   // 0–255 (safe speed for testing)
void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  Serial.println("Motor Direction Calibration Started");
  // Run both motors forward
  runForward();
}
void loop() {
}
//////////////////// RUN FORWARD ////////////////////
void runForward() {
  // Motor A forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, CALIB_SPEED);
  // Motor B forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, CALIB_SPEED);
}