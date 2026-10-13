#pragma once

// Deciding what a ride profile means, and what the controller is told.
//
// Pure and host-testable on purpose: controller_backends.cpp owns the UART and
// decides WHEN bytes go out, this module decides WHAT they say. Nothing here
// touches a stream or the app state.
//
// Every profile field is written with a LispBM `conf-set`, the same mechanism
// the 1zun4/vesc_scooter_support package uses for its speed modes:
//
//   l-max-erpm / l-min-erpm   the forward and reverse speed limits
//   l-current-max-scale       acceleration, as a fraction of the controller's
//                             configured motor current
//   l-current-min-scale       braking, likewise
//   foc-fw-current-max        field weakening
//   l-current-max             absolute motor current limit
//   l-in-current-max          absolute battery current limit
//
// `conf-set` writes into the controller's RAM and never its flash, and unlike
// COMM_SET_MCCONF_TEMP it changes only the fields named, so the limits a profile
// does not own cannot be widened by omission. The firmware rate-limits the Lisp
// REPL to one command per 0.5 s, so the whole profile goes out as a single
// expression: a block of conf-set forms is one command.

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

// The most characters a generated expression can take. A VESC short packet
// carries at most 255 payload bytes, and the payload is the command id, the
// expression and a NUL, so the expression must stay under 253.
constexpr size_t kVescProfileLispMaxLen = 256;

// What a successful write is judged against, in the controller's own units. The
// scales are fractions, matching COMM_GET_MCCONF_TEMP.
struct VescProfileTarget {
  float currentMinScale;
  float currentMaxScale;
  float minErpm;
  float maxErpm;
  float inCurrentMax;
};

// Converts a speed limit to the signed ERPM the controller expects. Forward
// limits are positive and reverse limits negative; `rpmPerKmh` is the conversion
// factor the telemetry task computes from the wheel, pole pairs, gear ratio and
// speed calibration. Returns 0 for a non-positive factor, so a limit the display
// cannot compute is left alone rather than sent as nonsense.
int32_t vescProfileErpmForSpeed(uint16_t speedKmh, bool reverse, float rpmPerKmh);

// The single LispBM expression that sets the whole profile, with the values
// clamped to what a controller accepts. Written into `out`, which must hold at
// least kVescProfileLispMaxLen bytes. Returns the character count, or 0 when the
// buffer is too small. A non-positive `rpmPerKmh` leaves the two speed limits out
// rather than setting them to zero, which would clamp the motor to a standstill.
size_t vescProfileLisp(uint16_t maxSpeedKmh, uint16_t reverseSpeedKmh, uint16_t accelPercent,
                       uint16_t brakePercent, uint16_t fieldWeakeningAmps, uint16_t motorAmps,
                       uint16_t batteryAmps, float rpmPerKmh, char *out, size_t cap);

// The target a profile asks the controller to report back, in the controller's
// own units. Scale inputs are percentages (0..100).
VescProfileTarget vescProfileTarget(uint16_t maxSpeedKmh, uint16_t reverseSpeedKmh,
                                    uint16_t accelPercent, uint16_t brakePercent,
                                    uint16_t batteryAmps, float rpmPerKmh);

// Compares a controller reply against the target. The tolerances absorb the
// rounding the firmware itself applies (it stores the scales in coarse steps and
// clamps ERPM to hardware limits). Fields the profile left alone -- a speed limit
// the display could not convert -- are not compared.
bool vescProfileReadBackMatches(const VescProfileTarget &target, const VescMcconfTemp &reply);

// The inverse of vescProfileErpmForSpeed, for showing what the controller
// reports in the pages' own units.
uint16_t vescProfileSpeedFromErpm(int32_t erpm, float rpmPerKmh);

// Converts a scale fraction back to a percentage for display, clamped to 0..100.
uint16_t vescProfileScalePercent(float scale);
