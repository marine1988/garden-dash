#pragma once
#include <stdint.h>

// Display-side cruise control for a VESC controller.
//
// The VESC's own speed PID holds the set eRPM once COMM_SET_RPM arrives, so the
// display never runs a speed loop of its own: it decides when cruise is active
// and what eRPM to ask for, from telemetry it already has, and re-sends the
// command often enough that the controller's command timeout -- which releases
// the motor -- never expires. Everything here is free of Arduino and LVGL, so
// the host tests drive the state machine directly.
//
// Safety, in the order it is applied each cycle: a fault, a dead link, or the
// rider's own cancel always disengages; the rider cannot engage below the
// minimum speed or in a ride mode the profile does not allow; and dropping back
// under the minimum speed (hard braking, a stop, a hill start) releases the
// controller. The display never commands a value it cannot see a reason for, and
// it releases rather than coasts when it stops commanding.

// Ride modes the controller can report in telemetry, 1..3, matching the three
// configurable speed modes. 0 means the controller does not report one.
enum CruiseRideMode : uint8_t {
  CRUISE_MODE_NONE = 0,
  CRUISE_MODE_ECO = 1,
  CRUISE_MODE_DRIVE = 2,
  CRUISE_MODE_SPORT = 3,
};

struct CruiseSettings {
  bool enabled;
  uint16_t holdTimeDeciS;       // press-and-hold to engage, tenths of a second
  uint16_t minSpeedKmh;         // below this, cruise refuses to engage
  uint16_t maxSpeedKmh;         // ceiling the target is clamped to
  uint16_t deadbandHundredths;  // speed error left to the controller, 0.01 km/h
  uint8_t allowedModes;         // bit (1 << (mode-1)); 0 refuses every reported mode
};

enum CruiseState : uint8_t {
  CRUISE_STATE_OFF,
  CRUISE_STATE_ACTIVE,
  CRUISE_STATE_BLOCKED,  // an engage request was refused (too slow, mode not allowed)
};

// What the caller should send to the controller this cycle.
struct CruiseOutput {
  bool send;        // true when a control command should go out
  uint8_t command;  // a VESC_COMM_SET_* id
  int32_t value;    // the command argument (eRPM for SET_RPM)
  CruiseState state;
};

// Inputs the telemetry task already has for this cycle.
struct CruiseInputs {
  float speedKmh;   // display-side speed, the same figure the dashboard shows
  float rpm;        // controller eRPM, signed
  float rpmPerKmh;  // eRPM per km/h, from the drivetrain setup; <= 0 disables engage
  uint8_t faultCode;
  uint8_t rideMode;  // 1..3, 0 unknown
  bool linkLive;
};

// Runtime state, owned by the telemetry task.
struct CruiseRuntime {
  CruiseState state;
  uint16_t targetKmh;
  int32_t targetRpm;
  float rpmPerKmh;  // captured at engage so a target nudge needs no re-derivation
  uint32_t lastCommandMs;
};

// Applies one cycle and returns what, if anything, to send. `requestEngage` is
// true on the cycle the rider asked to engage, `requestDisengage` on the one
// they asked to stop. Called every telemetry poll; the heartbeat period is
// enforced inside.
CruiseOutput cruiseControlStep(CruiseRuntime &rt, const CruiseSettings &settings, const CruiseInputs &in,
                               bool requestEngage, bool requestDisengage, uint32_t nowMs);

// Nudges the active target by `deltaKmh`, clamped to the settings window, and
// returns the new target in km/h. Returns the unchanged target when cruise is
// not active.
uint16_t cruiseControlAdjustTarget(CruiseRuntime &rt, const CruiseSettings &settings, int deltaKmh);

// True when the reported ride mode is allowed by the settings. An unknown mode
// (0) or a mask of 0 does not block: the display only refuses a mode the rider
// explicitly excluded.
bool cruiseModeAllowed(const CruiseSettings &settings, uint8_t mode);

// How often the command must be re-sent to stay inside the controller's command
// timeout. 20 Hz is comfortably inside the 1 s default while leaving the poll
// loop free for telemetry.
constexpr uint32_t kCruiseCommandPeriodMs = 50;

// The argument ZERO for a release: COMM_SET_CURRENT 0 A leaves the motor
// coasting with no torque, which is the safe way to hand control back the
// instant cruise is cancelled rather than waiting out the controller's timeout.
constexpr int32_t kCruiseReleaseValue = 0;
