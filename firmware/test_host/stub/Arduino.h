// Just enough Arduino for the PC tests in test_host/: a clock the test drives.
#pragma once
#include <math.h>
#include <stdint.h>
#include <stddef.h>

extern unsigned long g_fake_us;
inline unsigned long micros() { return g_fake_us; }
inline unsigned long millis() { return g_fake_us / 1000; }
