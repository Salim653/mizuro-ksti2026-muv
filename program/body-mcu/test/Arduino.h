#pragma once
/*
 * Mock Arduino API untuk tes host (g++) — BUKAN untuk build board.
 *
 * Cukup untuk body_controller.ino: millis(), Serial, pin I/O, constrain.
 * Semua state mock bisa diperiksa/di-set langsung dari tes.
 */
#include <cstdint>
#include <cstddef>
#include <string>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// Pin analog (nama AVR) — dipakai cabang non-STM32 di sketsa.
constexpr int A0 = 14;
constexpr int A1 = 15;
constexpr int A2 = 16;
constexpr int A3 = 17;
constexpr int A4 = 18;
constexpr int A5 = 19;

// Waktu mock: tes yang mengendalikan, bukan wall clock.
extern uint32_t mockMillis;
inline uint32_t millis() { return mockMillis; }

constexpr int MOCK_PIN_COUNT = 64;
extern int mockPinMode[MOCK_PIN_COUNT];
extern int mockDigitalOut[MOCK_PIN_COUNT];
extern int mockAnalogOut[MOCK_PIN_COUNT];
extern int mockDigitalIn[MOCK_PIN_COUNT];

inline void pinMode(int pin, int mode) {
  if (pin >= 0 && pin < MOCK_PIN_COUNT) mockPinMode[pin] = mode;
}
inline void digitalWrite(int pin, int value) {
  if (pin >= 0 && pin < MOCK_PIN_COUNT) mockDigitalOut[pin] = value;
}
inline int digitalRead(int pin) {
  return (pin >= 0 && pin < MOCK_PIN_COUNT) ? mockDigitalIn[pin] : LOW;
}
inline void analogWrite(int pin, int value) {
  if (pin >= 0 && pin < MOCK_PIN_COUNT) mockAnalogOut[pin] = value;
}

template <typename T>
inline T constrain(T value, T low, T high) {
  return value < low ? low : (value > high ? high : value);
}

// Serial mock: `in` = bytes yang "diterima" MCU, `out` = yang ditulis MCU.
class MockSerial {
 public:
  std::string in;
  std::string out;
  size_t inPos = 0;

  void begin(unsigned long) {}
  int available() { return static_cast<int>(in.size() - inPos); }
  int read() {
    return inPos < in.size() ? static_cast<unsigned char>(in[inPos++]) : -1;
  }
  void print(const char *text) { out += text; }
  void print(char c) { out += c; }
  void write(const char *text) { out += text; }
  void flush() {}
};

extern MockSerial Serial;
