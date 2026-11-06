/*
 * Tes host untuk body_controller.ino — TANPA hardware, TANPA Arduino IDE.
 *
 * File ini meng-include sketsa langsung dan mengganti Arduino API dengan
 * mock (mock_arduino.h / mock_servo.h), jadi logika firmware yang sama
 * berjalan di laptop: parser, watchdog, e-stop, mixing skid, pemetaan
 * ESC/servo, dan format baris status.
 *
 * Jalankan: ./run_tests.sh   (butuh g++, C++17)
 *
 * Yang TIDAK diuji di sini: timing hardware sungguhan (PWM 50 Hz servo,
 * arus L298N), kalibrasi netral ESC di ESC sungguhan, dan compile untuk
 * board (arduino-cli di Jetson — lihat README).
 */
#include "../body_controller.ino"

#include <cstdio>
#include <cstring>
#include <string>

// ---------------------------------------------------------------------------
// Definisi state mock (extern di mock_arduino.h)
// ---------------------------------------------------------------------------
uint32_t mockMillis = 0;
int mockPinMode[MOCK_PIN_COUNT];
int mockDigitalOut[MOCK_PIN_COUNT];
int mockAnalogOut[MOCK_PIN_COUNT];
int mockDigitalIn[MOCK_PIN_COUNT];
MockSerial Serial;

static int checks = 0;
static int failures = 0;

#define CHECK(condition)                                                \
  do {                                                                  \
    ++checks;                                                           \
    if (!(condition)) {                                                 \
      ++failures;                                                       \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);  \
    }                                                                   \
  } while (0)

// ---------------------------------------------------------------------------
// Helper
// ---------------------------------------------------------------------------
static void resetFirmware() {
  mockMillis = 0;
  Serial.in.clear();
  Serial.out.clear();
  Serial.inPos = 0;
  for (int i = 0; i < MOCK_PIN_COUNT; ++i) {
    mockPinMode[i] = 0;
    mockDigitalOut[i] = 0;
    mockAnalogOut[i] = 0;
    mockDigitalIn[i] = HIGH;
  }
  escServo = Servo();
  steerServo = Servo();
  rxOk = 0;
  targetSpeedMps = 0.0f;
  targetSteerRad = 0.0f;
  appliedSpeed = 0.0f;
  appliedSteer = 0.0f;
  commandSeen = false;
  lastCommandMs = 0;
  lastControlMs = 0;
  lastStatusMs = 0;
  driverMode = DEFAULT_MODE;
  setup();
}

static void feed(const char *text) { Serial.in += text; }

/** Majukan waktu mock `ms` lalu jalankan satu iterasi loop(). */
static void tick(uint32_t ms) {
  mockMillis += ms;
  loop();
}

static std::string lastStatusLine() {
  const size_t pos = Serial.out.rfind("#v1,");
  if (pos == std::string::npos) return std::string();
  const size_t end = Serial.out.find('\n', pos);
  if (end == std::string::npos) return std::string();
  return Serial.out.substr(pos, end - pos);
}

static int statusLineCount() {
  int count = 0;
  size_t pos = 0;
  while ((pos = Serial.out.find("#v1,", pos)) != std::string::npos) {
    ++count;
    pos += 4;
  }
  return count;
}

// ---------------------------------------------------------------------------
// Boot & keselamatan dasar
// ---------------------------------------------------------------------------
static void test_boot_output_safe() {
  resetFirmware();
  tick(0);   // loop pertama tanpa perintah apa pun
  CHECK(mockAnalogOut[MOTOR_FL_PWM_PIN] == 0);
  CHECK(mockAnalogOut[MOTOR_RL_PWM_PIN] == 0);
  CHECK(mockAnalogOut[MOTOR_FR_PWM_PIN] == 0);
  CHECK(mockAnalogOut[MOTOR_RR_PWM_PIN] == 0);
  CHECK(mockAnalogOut[MOTOR_PWM_PIN] == 0);
  CHECK(escServo.attachedPin == -1);     // mode default skid4: tanpa servo
  CHECK(steerServo.attachedPin == -1);
  CHECK(appliedSpeed == 0.0f);
  CHECK(statusLineCount() == 0);         // status baru setelah STATUS_PERIOD_MS
}

static void test_boot_status_stale() {
  resetFirmware();
  tick(100);
  CHECK(statusLineCount() == 1);
  CHECK(lastStatusLine() == "#v1,skid4,0,1,0.000,0.000,0,100");
}

static void test_setup_pin_modes() {
  resetFirmware();
  CHECK(mockPinMode[ESTOP_PIN] == INPUT_PULLUP);
  CHECK(mockPinMode[MOTOR_FL_PWM_PIN] == OUTPUT);
  CHECK(mockPinMode[MOTOR_FL_DIR_PIN] == OUTPUT);
  CHECK(mockPinMode[MOTOR_FR_PWM_PIN] == OUTPUT);
  CHECK(mockPinMode[MOTOR_FR_DIR_PIN] == OUTPUT);
  CHECK(mockPinMode[MOTOR_RL_PWM_PIN] == OUTPUT);
  CHECK(mockPinMode[MOTOR_RL_DIR_PIN] == OUTPUT);
  CHECK(mockPinMode[MOTOR_RR_PWM_PIN] == OUTPUT);
  CHECK(mockPinMode[MOTOR_RR_DIR_PIN] == OUTPUT);
  CHECK(mockPinMode[MOTOR_PWM_PIN] == OUTPUT);
  CHECK(mockPinMode[MOTOR_DIR_PIN] == OUTPUT);
}

// ---------------------------------------------------------------------------
// Parser setpoint
// ---------------------------------------------------------------------------
static void test_setpoint_drives_skid4() {
  resetFirmware();
  feed("0.4000,-0.1000\n");
  tick(10);
  // steer negatif = yaw kanan -> roda kiri lebih cepat
  CHECK(mockAnalogOut[MOTOR_FL_PWM_PIN] == 134);
  CHECK(mockAnalogOut[MOTOR_RL_PWM_PIN] == 134);
  CHECK(mockAnalogOut[MOTOR_FR_PWM_PIN] == 121);
  CHECK(mockAnalogOut[MOTOR_RR_PWM_PIN] == 121);
  CHECK(mockDigitalOut[MOTOR_FL_DIR_PIN] == HIGH);
  CHECK(mockDigitalOut[MOTOR_FR_DIR_PIN] == HIGH);
  CHECK(appliedSpeed == 0.4f);
  CHECK(appliedSteer == -0.1f);
  CHECK(rxOk == 1);
  CHECK(statusLineCount() == 0);
  tick(100);   // t=110: tepat 100 ms setelah perintah -> masih fresh
  CHECK(lastStatusLine() == "#v1,skid4,0,0,0.400,-0.100,1,110");
  CHECK(mockAnalogOut[MOTOR_FL_PWM_PIN] == 134);
}

static void test_setpoint_rejects_malformed() {
  resetFirmware();
  feed("0.4\n");               // tanpa koma
  feed("abc,0.1\n");           // bukan angka
  feed("0.4,0.1x\n");          // sampah di ujung
  feed("0.9,0.1\n");           // speed di luar batas
  feed("0.4,0.6\n");           // steer di luar batas
  feed("nan,0.1\n");           // NaN
  feed("inf,0.1\n");           // inf
  feed(",0.1\n");              // speed kosong
  feed("0.4,\n");              // steer kosong
  feed("\n");                  // baris kosong
  feed("0.4,0.1,0.2\n");       // tiga field
  tick(10);
  CHECK(rxOk == 0);
  CHECK(appliedSpeed == 0.0f);
  CHECK(mockAnalogOut[MOTOR_FL_PWM_PIN] == 0);
  // parser tetap hidup setelah sampah
  feed("0.3000,0.0500\n");
  tick(0);
  CHECK(rxOk == 1);
  CHECK(appliedSpeed == 0.3f);
  CHECK(appliedSteer == 0.05f);
}

static void test_line_overflow_resync() {
  resetFirmware();
  feed(std::string(200, 'A').c_str());
  feed("\n");
  feed("0.4000,0.0000\n");
  tick(10);
  CHECK(rxOk == 1);
  CHECK(appliedSpeed == 0.4f);
}

// ---------------------------------------------------------------------------
// Watchdog & e-stop
// ---------------------------------------------------------------------------
static void test_watchdog_trips() {
  resetFirmware();
  feed("0.4000,0.0000\n");
  tick(10);
  CHECK(mockAnalogOut[MOTOR_FL_PWM_PIN] == 128);
  tick(100);   // t=110: tepat 100 ms -> masih fresh
  CHECK(mockAnalogOut[MOTOR_FL_PWM_PIN] == 128);
  tick(1);     // t=111: 101 ms -> watchdog trip
  CHECK(mockAnalogOut[MOTOR_FL_PWM_PIN] == 0);
  CHECK(mockAnalogOut[MOTOR_FR_PWM_PIN] == 0);
  CHECK(appliedSpeed == 0.0f);
  tick(99);    // t=210: status berikutnya melaporkan stale=1
  CHECK(lastStatusLine() == "#v1,skid4,0,1,0.000,0.000,1,210");
}

static void test_estop_cuts_output() {
  resetFirmware();
  feed("0.4000,0.0000\n");
  tick(10);
  CHECK(mockAnalogOut[MOTOR_FL_PWM_PIN] == 128);
  mockDigitalIn[ESTOP_PIN] = LOW;   // e-stop active-low
  tick(5);
  CHECK(mockAnalogOut[MOTOR_FL_PWM_PIN] == 0);
  CHECK(appliedSpeed == 0.0f);
  tick(95);   // t=110
  // estop=1 memotong output; stale=1 karena safeStop() butuh perintah
  // BARU untuk resume (tidak auto-resume setelah e-stop dilepas).
  CHECK(lastStatusLine() == "#v1,skid4,1,1,0.000,0.000,1,110");
  mockDigitalIn[ESTOP_PIN] = HIGH;  // lepas, perintah segar -> jalan lagi
  feed("0.4000,0.0000\n");
  tick(0);
  CHECK(mockAnalogOut[MOTOR_FL_PWM_PIN] == 128);
}

// ---------------------------------------------------------------------------
// Mode output
// ---------------------------------------------------------------------------
static void test_mode_switch_esc1_and_arming() {
  resetFirmware();
  feed("#M,esc1\n");
  tick(0);
  CHECK(driverMode == DriverMode::Esc1);
  CHECK(escServo.attachedPin == ESC_SIGNAL_PIN);
  CHECK(steerServo.attachedPin == STEER_SIGNAL_PIN);
  feed("0.4000,0.1000\n");
  tick(10);   // t=10: ESC masih arming -> tahan netral
  CHECK(escServo.lastPulse == ESC_NEUTRAL_US);
  CHECK(steerServo.lastPulse == STEER_CENTER_US);
  CHECK(appliedSpeed == 0.0f);
  tick(100);  // t=110
  CHECK(lastStatusLine() == "#v1,esc1,0,0,0.000,0.000,1,110");
  feed("0.4000,0.1000\n");
  tick(1990); // t=2100: arming selesai (>= ESC_ARM_MS)
  CHECK(escServo.lastPulse == 1770);      // 1500+40+230
  CHECK(steerServo.lastPulse == 1573);    // 1500+73
  CHECK(appliedSpeed == 0.4f);
  tick(100);  // t=2200
  CHECK(lastStatusLine() == "#v1,esc1,0,0,0.400,0.100,2,2200");
  // kembali ke skid4: servo dilepas
  feed("#M,skid4\n");
  tick(0);
  CHECK(driverMode == DriverMode::Skid4);
  CHECK(escServo.attachedPin == -1);
  CHECK(steerServo.attachedPin == -1);
}

static void test_mode_switch_hb1() {
  resetFirmware();
  feed("#M,hb1\n");
  tick(0);
  CHECK(driverMode == DriverMode::Hb1);
  CHECK(escServo.attachedPin == -1);
  CHECK(steerServo.attachedPin == STEER_SIGNAL_PIN);
  feed("0.4000,0.1000\n");
  tick(10);
  CHECK(mockAnalogOut[MOTOR_PWM_PIN] == 128);
  CHECK(mockDigitalOut[MOTOR_DIR_PIN] == HIGH);
  CHECK(steerServo.lastPulse == 1573);
  feed("-0.4000,0.0000\n");
  tick(95);   // t=105: masih fresh (95 ms)
  CHECK(mockAnalogOut[MOTOR_PWM_PIN] == 128);
  CHECK(mockDigitalOut[MOTOR_DIR_PIN] == LOW);
  CHECK(steerServo.lastPulse == 1500);
}

static void test_invalid_mode_ignored() {
  resetFirmware();
  feed("#M,nope\n");
  feed("#M,\n");
  feed("#Z,skid4\n");
  feed("#v1,skid4,0,0,0.000,0.000,0,0\n");   // status yang memantul
  tick(10);
  CHECK(driverMode == DriverMode::Skid4);
  CHECK(escServo.attachedPin == -1);
  CHECK(rxOk == 0);
}

// ---------------------------------------------------------------------------
// Pemetaan output (unit langsung)
// ---------------------------------------------------------------------------
static void test_esc_pulse_mapping() {
  CHECK(escPulse(0.0f) == ESC_NEUTRAL_US);
  CHECK(escPulse(0.01f) == ESC_NEUTRAL_US);    // di bawah MIN_COMMAND_MPS
  CHECK(escPulse(-0.01f) == ESC_NEUTRAL_US);
  CHECK(escPulse(0.8f) == ESC_MAX_US);
  CHECK(escPulse(-0.8f) == ESC_MIN_US);
  CHECK(escPulse(0.05f) == 1569);              // 1500+40+29
  CHECK(escPulse(-0.05f) == 1431);             // 1500-40-29
  // inti jebakan resolusi throttle: perintah kecil HARUS lewat deadband
  CHECK(escPulse(0.05f) > ESC_NEUTRAL_US + ESC_DEADBAND_US);
  CHECK(escPulse(-0.05f) < ESC_NEUTRAL_US - ESC_DEADBAND_US);
  CHECK(escPulse(0.4f) == 1770);               // 1500+40+230
}

static void test_steer_pulse_mapping() {
  CHECK(steerPulse(0.0f) == 1500);
  CHECK(steerPulse(0.55f) == STEER_MAX_US);
  CHECK(steerPulse(-0.55f) == STEER_MIN_US);
  CHECK(steerPulse(0.275f) == 1700);
  CHECK(steerPulse(0.1f) == 1573);
  CHECK(steerPulse(-0.1f) == 1427);
  CHECK(steerPulse(5.0f) == STEER_MAX_US);     // di-clamp
}

static void test_skid_clamp_preserves_ratio() {
  resetFirmware();
  feed("0.8000,0.5500\n");
  tick(10);
  // left=0.69 right=0.91 -> peak 0.91 diskalakan ke 0.8; rasio belok tetap
  CHECK(mockAnalogOut[MOTOR_FL_PWM_PIN] == 193);
  CHECK(mockAnalogOut[MOTOR_RL_PWM_PIN] == 193);
  CHECK(mockAnalogOut[MOTOR_FR_PWM_PIN] == 255);
  CHECK(mockAnalogOut[MOTOR_RR_PWM_PIN] == 255);
  CHECK(mockDigitalOut[MOTOR_FL_DIR_PIN] == HIGH);
  CHECK(mockDigitalOut[MOTOR_FR_DIR_PIN] == HIGH);
}

static void test_skid_pivot_in_place() {
  resetFirmware();
  feed("0.0000,0.5500\n");
  tick(10);
  // speed 0 + steer penuh = putar di tempat: kiri mundur, kanan maju
  CHECK(mockAnalogOut[MOTOR_FL_PWM_PIN] == 35);
  CHECK(mockAnalogOut[MOTOR_RL_PWM_PIN] == 35);
  CHECK(mockAnalogOut[MOTOR_FR_PWM_PIN] == 35);
  CHECK(mockAnalogOut[MOTOR_RR_PWM_PIN] == 35);
  CHECK(mockDigitalOut[MOTOR_FL_DIR_PIN] == LOW);
  CHECK(mockDigitalOut[MOTOR_FR_DIR_PIN] == HIGH);
}

// ---------------------------------------------------------------------------
// Baris status
// ---------------------------------------------------------------------------
static void test_status_rx_ok_counts_valid_only() {
  resetFirmware();
  feed("0.1000,0.0000\n0.2000,0.0000\n0.9000,0.0000\nabc\n");
  tick(100);
  CHECK(rxOk == 2);   // yang di luar batas dan sampah tidak dihitung
  CHECK(lastStatusLine() == "#v1,skid4,0,0,0.200,0.000,2,100");
}

static void test_status_rate() {
  resetFirmware();
  tick(100);
  CHECK(statusLineCount() == 1);
  tick(50);
  CHECK(statusLineCount() == 1);   // 50 ms < STATUS_PERIOD_MS
  tick(50);
  CHECK(statusLineCount() == 2);
}

// ---------------------------------------------------------------------------
// Formatter angka (tanpa printf %f)
// ---------------------------------------------------------------------------
static void test_append_fixed() {
  char buf[32];
  size_t pos;

  pos = 0; buf[0] = '\0';
  appendFixed(buf, sizeof(buf), pos, 0.8f, 3);
  CHECK(std::string(buf) == "0.800");

  pos = 0; buf[0] = '\0';
  appendFixed(buf, sizeof(buf), pos, -0.55f, 3);
  CHECK(std::string(buf) == "-0.550");

  pos = 0; buf[0] = '\0';
  appendFixed(buf, sizeof(buf), pos, -0.0004f, 3);
  CHECK(std::string(buf) == "0.000");   // tidak pernah "-0.000"

  pos = 0; buf[0] = '\0';
  appendFixed(buf, sizeof(buf), pos, 12.0f, 1);
  CHECK(std::string(buf) == "12.0");

  pos = 0; buf[0] = '\0';
  appendFixed(buf, sizeof(buf), pos, 0.0f, 3);
  CHECK(std::string(buf) == "0.000");

  // kapasitas kecil: dipotong rapi, tetap null-terminated, tidak overflow
  char tiny[6];
  pos = 0; tiny[0] = '\0';
  appendFixed(tiny, sizeof(tiny), pos, -0.555f, 3);
  CHECK(std::strlen(tiny) < sizeof(tiny));
  CHECK(std::string(tiny) == "-0.55");
}

int main() {
  test_boot_output_safe();
  test_boot_status_stale();
  test_setup_pin_modes();
  test_setpoint_drives_skid4();
  test_setpoint_rejects_malformed();
  test_line_overflow_resync();
  test_watchdog_trips();
  test_estop_cuts_output();
  test_mode_switch_esc1_and_arming();
  test_mode_switch_hb1();
  test_invalid_mode_ignored();
  test_esc_pulse_mapping();
  test_steer_pulse_mapping();
  test_skid_clamp_preserves_ratio();
  test_skid_pivot_in_place();
  test_status_rx_ok_counts_valid_only();
  test_status_rate();
  test_append_fixed();

  std::printf("%d checks, %d failures\n", checks, failures);
  if (failures != 0) {
    return 1;
  }
  std::printf("body_controller host tests passed\n");
  return 0;
}
