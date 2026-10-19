#include <Arduino.h>
#include <Servo.h>

constexpr uint8_t MOTOR_PWM_PIN = 5;
constexpr uint8_t MOTOR_DIR_PIN = 4;
constexpr uint8_t STEERING_PIN = 6;
constexpr uint8_t ESTOP_PIN = 7;
constexpr uint32_t WATCHDOG_MS = 100;
constexpr uint32_t CONTROL_PERIOD_MS = 10;
constexpr int MOTOR_PWM_MAX = 255;
constexpr int STEER_CENTER_US = 1500;
constexpr int STEER_MIN_US = 1100;
constexpr int STEER_MAX_US = 1900;
constexpr float MAX_SPEED_MPS = 0.8f;
constexpr float MAX_STEER_RAD = 0.55f;

Servo steering;
uint32_t lastCommandMs = 0;
uint32_t lastControlMs = 0;
float targetSpeedMps = 0.0f;
float targetSteerRad = 0.0f;
bool commandSeen = false;

void safeStop() {
  analogWrite(MOTOR_PWM_PIN, 0);
  targetSpeedMps = 0.0f;
  targetSteerRad = 0.0f;
  steering.writeMicroseconds(STEER_CENTER_US);
  commandSeen = false;
}

void setMotor(float speedMps) {
  speedMps = constrain(speedMps, -MAX_SPEED_MPS, MAX_SPEED_MPS);
  digitalWrite(MOTOR_DIR_PIN, speedMps >= 0.0f ? HIGH : LOW);
  const int pwm = static_cast<int>(fabsf(speedMps) / MAX_SPEED_MPS * MOTOR_PWM_MAX);
  analogWrite(MOTOR_PWM_PIN, pwm);
}

void acceptLine(char *line) {
  char *comma = strchr(line, ',');
  if (comma == nullptr) return;
  *comma = '\0';
  char *endSpeed = nullptr;
  char *endSteer = nullptr;
  const float speed = strtof(line, &endSpeed);
  const float steer = strtof(comma + 1, &endSteer);
  if (endSpeed == line || *endSpeed != '\0' || endSteer == comma + 1 || *endSteer != '\0') return;
  if (!isfinite(speed) || !isfinite(steer) || fabsf(speed) > MAX_SPEED_MPS || fabsf(steer) > MAX_STEER_RAD) return;
  targetSpeedMps = speed;
  targetSteerRad = steer;
  lastCommandMs = millis();
  commandSeen = true;
}

void readCommands() {
  static char line[96];
  static size_t used = 0;
  while (Serial.available()) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\n') {
      line[used] = '\0';
      acceptLine(line);
      used = 0;
    } else if (ch == '\r') {
      continue;
    } else if (used + 1 < sizeof(line)) {
      line[used++] = ch;
    } else {
      used = 0;
    }
  }
}

void setup() {
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  steering.attach(STEERING_PIN);
  Serial.begin(115200);
  safeStop();
  lastCommandMs = millis();
}

void loop() {
  readCommands();
  const uint32_t now = millis();
  const bool estopPressed = digitalRead(ESTOP_PIN) == LOW;
  const bool stale = !commandSeen || now - lastCommandMs > WATCHDOG_MS;
  if (estopPressed || stale) {
    safeStop();
  } else if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    setMotor(targetSpeedMps);
    const int pulse = STEER_CENTER_US + static_cast<int>(targetSteerRad / MAX_STEER_RAD *
                       (STEER_MAX_US - STEER_CENTER_US));
    steering.writeMicroseconds(constrain(pulse, STEER_MIN_US, STEER_MAX_US));
  }
}
