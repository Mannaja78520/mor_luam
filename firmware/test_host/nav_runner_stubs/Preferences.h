#pragma once
#include <cstddef>
#include <cstdint>
class Preferences {
public:
    void begin(const char*, bool) {}
    uint8_t getUChar(const char*, uint8_t fallback) { return fallback; }
    size_t getBytesLength(const char*) { return 0; }
    void getBytes(const char*, void*, size_t) {}
    void putUChar(const char*, uint8_t) {}
    void putBytes(const char*, const void*, size_t) {}
};
