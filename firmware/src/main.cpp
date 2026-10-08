// mor_luam firmware entry point. Everything else is in app/App.cpp;
// the map of the source tree is in ../../CLAUDE.md.
#include <Arduino.h>
#include "app/App.h"

static App app;

void setup() { app.begin(); }
void loop() { app.loop(); }
