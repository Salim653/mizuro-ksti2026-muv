#pragma once
/*
 * Mock Servo untuk tes host — merekam pin attach dan pulse terakhir.
 */
#include <cstdint>

class MockServo {
 public:
  int attachedPin = -1;
  int lastPulse = -1;
  int pulseCount = 0;

  void attach(int pin) { attachedPin = pin; }
  void detach() { attachedPin = -1; }
  bool attached() const { return attachedPin >= 0; }
  void writeMicroseconds(int us) {
    lastPulse = us;
    ++pulseCount;
  }
  int readMicroseconds() const { return lastPulse; }
};

#define Servo MockServo
