#pragma once
// Small host adapters for the actual WaypointRunner.cpp state-machine tests.
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <string>
extern uint32_t g_nav_ms;
inline uint32_t millis() { return g_nav_ms; }
using SemaphoreHandle_t = void*;
constexpr int portMAX_DELAY = 0;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return reinterpret_cast<void*>(1); }
inline void xSemaphoreTake(SemaphoreHandle_t, int) {}
inline void xSemaphoreGive(SemaphoreHandle_t) {}
class String {
public:
    String() = default;
    String(const char* s) : value_(s ? s : "") {}
    String(int n) : value_(std::to_string(n)) {}
    String(unsigned int n) : value_(std::to_string(n)) {}
    String(unsigned long n) : value_(std::to_string(n)) {}
    const char* c_str() const { return value_.c_str(); }
    bool operator==(const char* s) const { return value_ == s; }
    bool operator!=(const char* s) const { return value_ != s; }
    String operator+(const String& s) const { return String((value_ + s.value_).c_str()); }
private:
    std::string value_;
};
inline String operator+(const char* left, const String& right) { return String(left) + right; }
