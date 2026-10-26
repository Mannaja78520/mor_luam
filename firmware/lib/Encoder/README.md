# Encoder Library

Encoder counts pulses from quadrature encoded signals, which are commonly available from rotary knobs, motor or shaft sensors and other position sensors. 

http://www.pjrc.com/teensy/td_libs_Encoder.html

http://www.youtube.com/watch?v=2puhIong-cs

![Encoder Knobs Demo](http://www.pjrc.com/teensy/td_libs_Encoder_1.jpg)

## Use in mor_luam

This is a bundled upstream library; the description and examples above refer to
that library. The current ESP32 drive wrapper in `firmware/lib/esp32_Encoder/` uses
`ESP32Encoder`, declared in `firmware/platformio.ini`, rather than this generic
`Encoder` implementation. Check the active wrapper before changing dependencies.

For project setup, Docker commands and the robot web app, start with the
[mor_luam README](../../../README.md). Run `docker\mor_luam.bat fw-test` from
the repository root for the maintained host suites; upstream hardware examples
in this folder are separate sketches.
