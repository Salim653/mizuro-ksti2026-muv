/*
 * MIZURO KSTI 2026 — body controller (firmware MCU bodi) v2
 * =========================================================
 *
 * Dibaca dulu: program/body-mcu/README.md (pin map, cara uji, kalibrasi).
 *
 * TIGA mode output (dipilih runtime lewat serial, atau compile-time
 * DEFAULT_MODE — lihat README):
 *
 *   skid4  PROTOTYPE: 4x L298N, satu motor per roda, skid steer.
 *          `speed_mps,steering_rad` dicampur di MCU jadi kecepatan roda
 *          kiri/kanan (e-diff). Tidak ada servo.
 *   esc1   KOMPETISI: 1x ESC brushless (servo-PWM 1000-2000 us) untuk
 *          roda belakang + servo kemudi Ackermann.
 *   hb1    BENCH: 1x motor lewat driver PWM+DIR + servo kemudi.
 *
 * PROTOKOL (115200 baud, ASCII, satu baris per paket):
 *
 *   Jetson -> MCU:
 *     speed_mps,steering_rad\n   setpoint (4 desimal, batas protokol)
 *     #M,<mode>\n                pilih mode: skid4 | esc1 | hb1
 *
 *   MCU -> Jetson (10 Hz):
 *     #v1,<mode>,<estop>,<stale>,<speed_out>,<steer_out>,<rx_ok>,<uptime_ms>\n
 *       mode      = mode output yang sedang aktif
 *       estop     = 1 kalau tombol E-stop tertekan (penggerak dipotong)
 *       stale     = 1 kalau watchdog trip (tidak ada setpoint segar)
 *       speed_out = kecepatan yang BENAR-BENAR diterapkan (m/s, 0 = dipotong)
 *       steer_out = sudut kemudi yang diterapkan (rad, 0 = dipotong)
 *       rx_ok     = jumlah setpoint valid yang diterima (uint32, wrap)
 *       uptime_ms = millis()
 *
 *   Catatan: di mode skid4, speed_out/steer_out adalah perintah SEBELUM
 *   mixing; kecepatan per-roda tidak dikirim (lihat README "yang belum
 *   ada"). Batas protokol SAMA dengan mizuro/protocol.py di sisi Jetson —
 *   ubah kedua sisi bersamaan.
 *
 * KONTRAK KESELAMATAN (yang tidak boleh dilanggar):
 *   1. Boot: semua output mati/netral sebelum ada setpoint valid.
 *   2. Watchdog: tidak ada setpoint segar dalam WATCHDOG_MS -> semua
 *      output mati. Loop Jetson WAJIB kirim setiap iterasi (heartbeat).
 *   3. E-stop (active-low, fisik) memotong output terlepas dari serial.
 *      Bukan pengganti pemutusan daya — lapisan terakhir saja.
 *   4. Setpoint di luar batas / baris rusak DITOLAK, output terakhir
 *      yang valid tetap dipakai sampai watchdog trip.
 *   5. Ganti mode selalu melewati safeStop() dulu.
 *
 * CATATAN PORTABILITAS: file ini harus compile di TIGA tempat —
 *   - host (g++ + test/mock_*.h) untuk tes tanpa hardware,
 *   - arduino:avr:uno (cek sintaks saja),
 *   - STM32F401/F411 Blackpill (target nyata, core stm32duino).
 * Karena itu: TIDAK ada printf %f (newlib-nano AVR tidak mendukung),
 * tidak ada String, tidak ada fitur khusus satu core. Angka desimal
 * diformat manual (appendFixed).
 */
#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Konstanta protokol (HARUS sama dengan mizuro/protocol.py)
// ---------------------------------------------------------------------------
constexpr uint32_t WATCHDOG_MS = 100;
constexpr uint32_t CONTROL_PERIOD_MS = 10;   // tick output 100 Hz
constexpr uint32_t STATUS_PERIOD_MS = 100;   // baris status 10 Hz
constexpr int MOTOR_PWM_MAX = 255;
constexpr float MAX_SPEED_MPS = 0.8f;
constexpr float MAX_STEER_RAD = 0.55f;

// ---------------------------------------------------------------------------
// Parameter mode skid4 (PROTOTYPE — kalibrasi di sasis nyata, lihat README)
// ---------------------------------------------------------------------------
constexpr float SKID_TRACK_M = 0.20f;      // jarak kiri-kanan roda (m)
constexpr float SKID_MAX_YAW_RATE = 2.0f;  // rad/s pada steer penuh

// ---------------------------------------------------------------------------
// Parameter servo kemudi (mode esc1/hb1)
// ---------------------------------------------------------------------------
constexpr int STEER_CENTER_US = 1500;
constexpr int STEER_MIN_US = 1100;
constexpr int STEER_MAX_US = 1900;

// ---------------------------------------------------------------------------
// Parameter ESC (mode esc1)
// ---------------------------------------------------------------------------
constexpr int ESC_NEUTRAL_US = 1500;
constexpr int ESC_MIN_US = 1000;
constexpr int ESC_MAX_US = 2000;
// Deadband netral ESC. PENTING (jebakan resolusi throttle): pada gir 1:10,
// 1 m/s = 1-3% throttle penuh = pulse 1515-1560 us, DI DALAM deadband ESC
// biasa. Window maju karena itu dijepit mulai netral+deadband supaya
// perintah kecil pun benar-benar menggerakkan roda.
constexpr int ESC_DEADBAND_US = 40;
constexpr float MIN_COMMAND_MPS = 0.02f;   // di bawah ini = netral
constexpr uint32_t ESC_ARM_MS = 2000;      // tahan netral setelah boot

// Polaritas arah L298N/driver: flip kalau kabel motor terbalik.
constexpr int MOTOR_DIR_FORWARD = HIGH;

enum class DriverMode : uint8_t { Skid4, Esc1, Hb1 };

// Mode saat boot sebelum Jetson mengirim #M,<mode>. Prototype = skid4.
constexpr DriverMode DEFAULT_MODE = DriverMode::Skid4;

// ---------------------------------------------------------------------------
// Pin map — PLACEHOLDER. Cocokkan dengan wiring sebelum flashing.
// Hinduki di Blackpill: PA9/PA10 (Serial), PA13/PA14 (SWD),
// PA11/PA12/PA15 (USB), PC14/PC15 (oscilator).
// ---------------------------------------------------------------------------
#if defined(ARDUINO_ARCH_STM32)
// Blackpill F401/F411 — pin PWM-capable (TIM), konfirmasi di datasheet.
constexpr int MOTOR_FL_PWM_PIN = PB6;   // roda depan kiri
constexpr int MOTOR_FL_DIR_PIN = PB7;
constexpr int MOTOR_FR_PWM_PIN = PB8;   // roda depan kanan
constexpr int MOTOR_FR_DIR_PIN = PB9;
constexpr int MOTOR_RL_PWM_PIN = PB3;   // roda belakang kiri
constexpr int MOTOR_RL_DIR_PIN = PB4;
constexpr int MOTOR_RR_PWM_PIN = PB5;   // roda belakang kanan
constexpr int MOTOR_RR_DIR_PIN = PB10;
constexpr int MOTOR_PWM_PIN = PB0;      // mode hb1: satu motor
constexpr int MOTOR_DIR_PIN = PB1;
constexpr int ESC_SIGNAL_PIN = PA0;     // mode esc1
constexpr int STEER_SIGNAL_PIN = PA1;   // mode esc1/hb1
constexpr int ESTOP_PIN = PB13;         // active-low, INPUT_PULLUP
#else
// Arduino Uno — HANYA untuk cek sintaks/build, bukan target.
constexpr int MOTOR_FL_PWM_PIN = 5;
constexpr int MOTOR_FL_DIR_PIN = 4;
constexpr int MOTOR_FR_PWM_PIN = 6;
constexpr int MOTOR_FR_DIR_PIN = 7;
constexpr int MOTOR_RL_PWM_PIN = 9;
constexpr int MOTOR_RL_DIR_PIN = 8;
constexpr int MOTOR_RR_PWM_PIN = 10;
constexpr int MOTOR_RR_DIR_PIN = 11;
constexpr int MOTOR_PWM_PIN = 3;
constexpr int MOTOR_DIR_PIN = 2;
constexpr int ESC_SIGNAL_PIN = A0;
constexpr int STEER_SIGNAL_PIN = A1;
constexpr int ESTOP_PIN = 12;
#endif

// ---------------------------------------------------------------------------
// State global
// ---------------------------------------------------------------------------
Servo steerServo;   // mode esc1/hb1
Servo escServo;     // mode esc1

DriverMode driverMode = DEFAULT_MODE;
uint32_t bootMs = 0;
uint32_t lastCommandMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastStatusMs = 0;
uint32_t rxOk = 0;              // setpoint valid diterima (wrap di 2^32)
float targetSpeedMps = 0.0f;
float targetSteerRad = 0.0f;
float appliedSpeed = 0.0f;      // output yang benar-benar diterapkan
float appliedSteer = 0.0f;
bool commandSeen = false;

// ---------------------------------------------------------------------------
// Format angka TANPA printf %f (newlib-nano AVR tidak mendukung %f).
// Semua append*: null-terminated, tidak pernah overflow kapasitas.
// ---------------------------------------------------------------------------
void appendText(char *buffer, size_t capacity, size_t &position, const char *text) {
  while (*text != '\0' && position + 1 < capacity) {
    buffer[position++] = *text++;
  }
  buffer[position] = '\0';
}

void appendLong(char *buffer, size_t capacity, size_t &position, long value) {
  char digits[24];
  size_t count = 0;
  const bool negative = value < 0;
  // Hindari UB pada LONG_MIN: pecah lewat unsigned.
  unsigned long magnitude = negative
      ? static_cast<unsigned long>(-(value + 1)) + 1UL
      : static_cast<unsigned long>(value);
  do {
    digits[count++] = static_cast<char>('0' + static_cast<int>(magnitude % 10));
    magnitude /= 10;
  } while (magnitude != 0 && count < sizeof(digits));
  if (negative && position + 1 < capacity) {
    buffer[position++] = '-';
  }
  while (count != 0 && position + 1 < capacity) {
    buffer[position++] = digits[--count];
  }
  buffer[position] = '\0';
}

void appendFixed(char *buffer, size_t capacity, size_t &position,
                 float value, int decimals) {
  const bool negative = value < 0.0f;
  const float magnitude = negative ? -value : value;
  long scale = 1;
  for (int i = 0; i < decimals; ++i) {
    scale *= 10;
  }
  const long rounded = static_cast<long>(
      magnitude * static_cast<float>(scale) + 0.5f);
  // rounded == 0 -> jangan cetak "-0.000"
  if (negative && rounded != 0 && position + 1 < capacity) {
    buffer[position++] = '-';
  }
  appendLong(buffer, capacity, position, rounded / scale);
  if (decimals <= 0) {
    return;
  }
  if (position + 1 < capacity) {
    buffer[position++] = '.';
  }
  long fraction = rounded % scale;
  long divisor = scale / 10;
  for (int i = 0; i < decimals; ++i) {
    if (position + 1 >= capacity || divisor == 0) {
      break;
    }
    buffer[position++] = static_cast<char>('0' + static_cast<int>((fraction / divisor) % 10));
    divisor /= 10;
  }
  buffer[position] = '\0';
}

const char *modeName(DriverMode mode) {
  switch (mode) {
    case DriverMode::Skid4: return "skid4";
    case DriverMode::Esc1: return "esc1";
    case DriverMode::Hb1: return "hb1";
  }
  return "?";
}

// ---------------------------------------------------------------------------
// Pemetaan output
// ---------------------------------------------------------------------------
void setHBridge(int pwmPin, int dirPin, float speedMps) {
  const float clamped = constrain(speedMps, -MAX_SPEED_MPS, MAX_SPEED_MPS);
  digitalWrite(dirPin, clamped >= 0.0f ? MOTOR_DIR_FORWARD
                                       : (MOTOR_DIR_FORWARD == HIGH ? LOW : HIGH));
  const int pwm = static_cast<int>(
      fabsf(clamped) / MAX_SPEED_MPS * static_cast<float>(MOTOR_PWM_MAX) + 0.5f);
  analogWrite(pwmPin, pwm);
}

int escPulse(float speedMps) {
  if (fabsf(speedMps) < MIN_COMMAND_MPS) {
    return ESC_NEUTRAL_US;
  }
  const float magnitude = fabsf(speedMps) / MAX_SPEED_MPS;   // 0..1
  const int span = (ESC_MAX_US - ESC_NEUTRAL_US) - ESC_DEADBAND_US;
  const int offset = ESC_DEADBAND_US +
      static_cast<int>(magnitude * static_cast<float>(span) + 0.5f);
  return speedMps > 0.0f ? ESC_NEUTRAL_US + offset
                         : ESC_NEUTRAL_US - offset;
}

int steerPulse(float steerRad) {
  const float ratio = constrain(steerRad, -MAX_STEER_RAD, MAX_STEER_RAD)
      / MAX_STEER_RAD;
  const float exact = static_cast<float>(STEER_CENTER_US) +
      ratio * static_cast<float>(STEER_MAX_US - STEER_CENTER_US);
  const int pulse = exact >= 0.0f
      ? static_cast<int>(exact + 0.5f)
      : static_cast<int>(exact - 0.5f);
  return constrain(pulse, STEER_MIN_US, STEER_MAX_US);
}

// Mode skid4: steering_rad = perintah yaw-rate ternormalisasi (steer penuh
// = SKID_MAX_YAW_RATE rad/s). Beda kecepatan kiri/kanan = yaw * track/2.
// Kalau puncak roda melebihi batas, dua sisi diskalakan bersama supaya
// rasio belok tetap (tidak pernah memotong satu sisi saja).
void applySkid4(float speedMps, float steerRad) {
  const float yaw = steerRad * SKID_MAX_YAW_RATE;
  const float delta = yaw * (SKID_TRACK_M * 0.5f);
  float left = speedMps - delta;
  float right = speedMps + delta;
  const float absLeft = fabsf(left);
  const float absRight = fabsf(right);
  const float peak = absLeft > absRight ? absLeft : absRight;
  if (peak > MAX_SPEED_MPS) {
    const float scale = MAX_SPEED_MPS / peak;
    left *= scale;
    right *= scale;
  }
  setHBridge(MOTOR_FL_PWM_PIN, MOTOR_FL_DIR_PIN, left);
  setHBridge(MOTOR_RL_PWM_PIN, MOTOR_RL_DIR_PIN, left);
  setHBridge(MOTOR_FR_PWM_PIN, MOTOR_FR_DIR_PIN, right);
  setHBridge(MOTOR_RR_PWM_PIN, MOTOR_RR_DIR_PIN, right);
}

void applyDrive(float speedMps) {
  if (driverMode == DriverMode::Esc1) {
    escServo.writeMicroseconds(escPulse(speedMps));
  } else {
    setHBridge(MOTOR_PWM_PIN, MOTOR_DIR_PIN, speedMps);
  }
}

// Tahan netral: dipakai saat ESC masih arming setelah boot.
void applyNeutral() {
  if (escServo.attached()) {
    escServo.writeMicroseconds(ESC_NEUTRAL_US);
  }
  if (steerServo.attached()) {
    steerServo.writeMicroseconds(STEER_CENTER_US);
  }
}

void safeStop() {
  // Semua channel motor dimatikan — termasuk channel mode lain supaya
  // pergantian mode tidak pernah meninggalkan output nyala.
  setHBridge(MOTOR_FL_PWM_PIN, MOTOR_FL_DIR_PIN, 0.0f);
  setHBridge(MOTOR_RL_PWM_PIN, MOTOR_RL_DIR_PIN, 0.0f);
  setHBridge(MOTOR_FR_PWM_PIN, MOTOR_FR_DIR_PIN, 0.0f);
  setHBridge(MOTOR_RR_PWM_PIN, MOTOR_RR_DIR_PIN, 0.0f);
  setHBridge(MOTOR_PWM_PIN, MOTOR_DIR_PIN, 0.0f);
  if (escServo.attached()) {
    escServo.writeMicroseconds(ESC_NEUTRAL_US);
  }
  if (steerServo.attached()) {
    steerServo.writeMicroseconds(STEER_CENTER_US);
  }
  targetSpeedMps = 0.0f;
  targetSteerRad = 0.0f;
  appliedSpeed = 0.0f;
  appliedSteer = 0.0f;
  commandSeen = false;
}

// ---------------------------------------------------------------------------
// Mode
// ---------------------------------------------------------------------------
bool parseMode(const char *text, DriverMode &mode) {
  if (strcmp(text, "skid4") == 0) {
    mode = DriverMode::Skid4;
    return true;
  }
  if (strcmp(text, "esc1") == 0) {
    mode = DriverMode::Esc1;
    return true;
  }
  if (strcmp(text, "hb1") == 0) {
    mode = DriverMode::Hb1;
    return true;
  }
  return false;
}

void attachForMode(DriverMode mode) {
  if (mode == DriverMode::Esc1) {
    escServo.attach(ESC_SIGNAL_PIN);
  } else {
    escServo.detach();
  }
  if (mode == DriverMode::Esc1 || mode == DriverMode::Hb1) {
    steerServo.attach(STEER_SIGNAL_PIN);
  } else {
    steerServo.detach();
  }
}

void setMode(DriverMode mode, bool force = false) {
  if (!force && mode == driverMode) {
    return;
  }
  driverMode = mode;
  attachForMode(mode);
  safeStop();   // mode baru selalu mulai dari output aman
}

// ---------------------------------------------------------------------------
// Parser serial
// ---------------------------------------------------------------------------
void acceptLine(char *line) {
  if (line[0] == '#') {
    // Perintah mode: #M,<mode>
    if (strncmp(line, "#M,", 3) != 0) {
      return;
    }
    DriverMode mode;
    if (!parseMode(line + 3, mode)) {
      return;   // mode tidak dikenal: abaikan, mode lama tetap jalan
    }
    setMode(mode);
    return;
  }

  // Setpoint: speed_mps,steering_rad — TOLAK apa pun yang tidak persis
  // dua angka terpisah koma (sampah serial tidak boleh jadi perintah).
  char *comma = strchr(line, ',');
  if (comma == nullptr) {
    return;
  }
  *comma = '\0';
  char *endSpeed = nullptr;
  char *endSteer = nullptr;
  const float speed = static_cast<float>(strtod(line, &endSpeed));
  const float steer = static_cast<float>(strtod(comma + 1, &endSteer));
  if (endSpeed == line || *endSpeed != '\0'
      || endSteer == comma + 1 || *endSteer != '\0') {
    return;
  }
  if (!isfinite(speed) || !isfinite(steer)
      || fabsf(speed) > MAX_SPEED_MPS || fabsf(steer) > MAX_STEER_RAD) {
    return;
  }
  targetSpeedMps = speed;
  targetSteerRad = steer;
  lastCommandMs = millis();
  commandSeen = true;
  ++rxOk;
}

void readCommands() {
  static char line[64];
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
      used = 0;   // baris terlalu panjang: buang, resinkronisasi di '\n'
    }
  }
}

// ---------------------------------------------------------------------------
// Status MCU -> Jetson
// ---------------------------------------------------------------------------
void emitStatus(uint32_t now, bool estop, bool stale) {
  char line[64];
  size_t position = 0;
  line[0] = '\0';
  appendText(line, sizeof(line), position, "#v1,");
  appendText(line, sizeof(line), position, modeName(driverMode));
  appendText(line, sizeof(line), position, ",");
  appendText(line, sizeof(line), position, estop ? "1" : "0");
  appendText(line, sizeof(line), position, ",");
  appendText(line, sizeof(line), position, stale ? "1" : "0");
  appendText(line, sizeof(line), position, ",");
  appendFixed(line, sizeof(line), position, appliedSpeed, 3);
  appendText(line, sizeof(line), position, ",");
  appendFixed(line, sizeof(line), position, appliedSteer, 3);
  appendText(line, sizeof(line), position, ",");
  appendLong(line, sizeof(line), position, static_cast<long>(rxOk));
  appendText(line, sizeof(line), position, ",");
  appendLong(line, sizeof(line), position, static_cast<long>(now));
  appendText(line, sizeof(line), position, "\n");
  Serial.print(line);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  pinMode(MOTOR_FL_PWM_PIN, OUTPUT);
  pinMode(MOTOR_FL_DIR_PIN, OUTPUT);
  pinMode(MOTOR_FR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_FR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RL_PWM_PIN, OUTPUT);
  pinMode(MOTOR_RL_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_RR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  bootMs = millis();
  setMode(DEFAULT_MODE, true);   // output aman sebelum ada perintah
}

void loop() {
  readCommands();

  const uint32_t now = millis();
  const bool estopPressed = digitalRead(ESTOP_PIN) == LOW;
  const bool stale = !commandSeen || (now - lastCommandMs > WATCHDOG_MS);
  // ESC butuh netral beberapa detik setelah power-on untuk arm.
  const bool arming = driverMode == DriverMode::Esc1
      && (now - bootMs < ESC_ARM_MS);

  if (estopPressed || stale) {
    safeStop();
  } else if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    const float speed = constrain(targetSpeedMps, -MAX_SPEED_MPS, MAX_SPEED_MPS);
    const float steer = constrain(targetSteerRad, -MAX_STEER_RAD, MAX_STEER_RAD);
    if (arming) {
      applyNeutral();   // ESC arming: tahan netral, perintah ditahan
    } else if (driverMode == DriverMode::Skid4) {
      applySkid4(speed, steer);
      appliedSpeed = speed;
      appliedSteer = steer;
    } else {
      applyDrive(speed);
      steerServo.writeMicroseconds(steerPulse(steer));
      appliedSpeed = speed;
      appliedSteer = steer;
    }
  }

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    emitStatus(now, estopPressed, stale);
  }
}
