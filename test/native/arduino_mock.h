// =============================================================================
// Minimal Arduino HAL mock for native (Mac/Linux/Windows) test execution.
// This provides fake implementations of Arduino functions so that
// logic-only tests can compile and run with g++/clang++ on a desktop.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>

// ---- Arduino constants ----
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

typedef uint8_t byte;
typedef bool boolean;

// ---- Simulated pin state ----
static int _pin_mode[70] = {};
static int _pin_state[70] = {};
static int _pwm_value[70] = {};

inline void pinMode(int pin, int mode)
{
    if (pin >= 0 && pin < 70)
        _pin_mode[pin] = mode;
}

inline void digitalWrite(int pin, int value)
{
    if (pin >= 0 && pin < 70)
        _pin_state[pin] = value;
}

inline int digitalRead(int pin)
{
    if (pin >= 0 && pin < 70)
        return _pin_state[pin];
    return LOW;
}

inline void analogWrite(int pin, int value)
{
    if (pin >= 0 && pin < 70)
        _pwm_value[pin] = value;
}

inline int analogRead(int pin)
{
    return 0;
}

inline void delay(unsigned long ms) { (void)ms; }
inline void delayMicroseconds(unsigned int us) { (void)us; }
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
inline long pulseIn(int pin, int state, unsigned long timeout = 1000000)
{
    (void)pin;
    (void)state;
    (void)timeout;
    return 35; // Simulate a mid-range green frequency reading
}

// ---- Minimal Serial mock ----
struct SerialMock
{
    void begin(long baud) { (void)baud; }
    void print(const char *s) { printf("%s", s); }
    void print(int v) { printf("%d", v); }
    void print(unsigned long v) { printf("%lu", v); }
    void println(const char *s) { printf("%s\n", s); }
    void println(int v) { printf("%d\n", v); }
    void println(unsigned long v) { printf("%lu\n", v); }
    void println() { printf("\n"); }
    operator bool() const { return true; }
};

static SerialMock Serial;

// ---- F() macro (no-op on native) ----
#define F(s) (s)

// ---- Helper to reset all mock state between tests ----
inline void resetMockPins()
{
    memset(_pin_mode, 0, sizeof(_pin_mode));
    memset(_pin_state, 0, sizeof(_pin_state));
    memset(_pwm_value, 0, sizeof(_pwm_value));
}

// ---- Helper to read back PWM values (not possible on real Arduino) ----
inline int getPWM(int pin)
{
    if (pin >= 0 && pin < 70)
        return _pwm_value[pin];
    return 0;
}
