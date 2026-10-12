#pragma once

// Deciding what a controller profile means, and what bytes carry it.
//
// Pure and host-testable on purpose: controller_backends.cpp owns the UART and
// decides WHEN bytes go out, this module decides WHAT they say. Nothing here
// touches a stream or the app state.
//
// VESC Tool's own profiles are the model: a forward and reverse speed limit and
// two current scales, plus the field-weakening current the scooter-support
// package writes alongside them. Speed and scales go out on
// COMM_SET_MCCONF_TEMP with store=0, which applies in the controller's RAM and
// never touches its flash. Field weakening has no field in that command, so it
// goes out as a LispBM expression over COMM_LISP_REPL_CMD -- which also applies
// in RAM, and restarts LispBM if it is not running.

#include <stddef.h>
#include <stdint.h>

#include "vesc_protocol.h"

// The three outcomes a write can report back to the UI.
enum VescProfileWriteStatus : uint8_t {
  VESC_PROFILE_WRITE_IDLE = 0,
  VESC_PROFILE_WRITE_SENT,       // request queued for the telemetry task
  VESC_PROFILE_WRITE_CONFIRMED,  // controller's reply matches the target
  VESC_PROFILE_WRITE_FAILED,     // link refused, timed out, or the reply disagreed
};

// The last write's outcome, plus the slot it refers to (0xFF when none). The UI
// polls this after asking for a write; the telemetry task writes it.
struct VescProfileWriteReport {
  VescProfileWriteStatus status;
  uint8_t index;
};

// A field weakening current above this is refused rather than sent: a typo here
// is applied to a live motor.
constexpr uint16_t kVescProfileMaxFieldWeakeningAmps = 60;

// What a successful write is judged against, in the controller's own units. The
// scales are fractions, matching COMM_GET_MCCONF_TEMP.
struct VescProfileTarget {
  float currentMinScale;
  float currentMaxScale;
  float minErpm;
  float maxErpm;
};

// Converts a speed limit to the signed ERPM the controller expects. Forward
// limits are positive and reverse limits negative; `rpmPerKmh` is the conversion
// factor the telemetry task already computes from the wheel, pole pairs, gear
// ratio and speed calibration. Returns 0 for a non-positive factor, so a limit
// the display cannot compute is left alone rather than sent as nonsense.
int32_t vescProfileErpmForSpeed(uint16_t speedKmh, bool reverse, float rpmPerKmh);

// The target a profile asks the controller to report back, in the controller's
// own units. Scale inputs are percentages (0..100).
VescProfileTarget vescProfileTarget(uint16_t forwardSpeedKmh, uint16_t reverseSpeedKmh,
                                    uint16_t currentMaxScalePercent, uint16_t currentMinScalePercent,
                                    float rpmPerKmh);

// The LispBM expression that sets the field-weakening current, clamped to
// kVescProfileMaxFieldWeakeningAmps. Written into `out`, which must hold at
// least kVescProfileLispMaxLen bytes. Returns the character count, or 0 when the
// buffer is too small.
size_t vescProfileLispFieldWeakening(uint16_t fieldWeakeningAmps, char *out, size_t cap);

// Compares a controller reply against the target. The tolerances absorb the
// rounding the firmware itself applies (it stores the scales in coarse steps and
// clamps ERPM to hardware limits), so a profile is not rejected over the
// controller's own precision.
bool vescProfileReadBackMatches(const VescProfileTarget &target, const VescMcconfTemp &reply);

// The inverse of vescProfileErpmForSpeed, for showing what the controller
// reports in the pages' own units.
uint16_t vescProfileSpeedFromErpm(int32_t erpm, float rpmPerKmh);

// Converts a scale fraction back to a percentage for display, clamped to 0..100.
uint16_t vescProfileScalePercent(float scale);

// The most characters a generated expression can take.
constexpr size_t kVescProfileLispMaxLen = 48;
