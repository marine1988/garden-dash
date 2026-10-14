#pragma once

// Diagnostics for the serial port, and the one build where there is no serial
// port to write them to.
//
// The default build keeps the VESC link on CN1 (GPIO22/GPIO27) and leaves the
// USB-serial pair (GPIO1/GPIO3) to the log, so these write to `Serial`.
//
// GARDEN_VESC_ON_USB_HEADER moves the link onto that pair, so that all four
// wires -- 5V, GND, TX, RX -- land on the power header and one connector carries
// the whole link. There the port *is* the link: printing into it would be read
// by the controller as protocol bytes, so every call compiles away. The
// controller must also be unpowered while the USB cable is plugged in, or its
// transmitter and the board's USB-serial chip push the same line. See
// docs/wiring.md.
//
// The host preview has no serial port either, so it compiles them away too.

#include "config.h"

#if defined(CYD_LVGL_PREVIEW) || GARDEN_VESC_ON_USB_HEADER

inline void gardenLog(const char *) {}
inline void gardenLogf(const char *, ...) {}

#else

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

inline void gardenLog(const char *line) { Serial.println(line); }

inline void gardenLogf(const char *format, ...) {
  char line[160];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  Serial.println(line);
}

#endif
