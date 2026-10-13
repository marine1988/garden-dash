// Host tests for the display-side cruise-control state machine.
//
// The module is pure logic, so these drive it directly: no controller, no
// radio, no LVGL. Each case fixes the settings and one cycle of inputs and
// checks what the dashboard would have sent, if anything. The output is a
// LispBM expression (the VESC ADC application holds the setpoint itself), not a
// periodic COMM_SET_RPM frame.

#include <cstdio>
#include <cstring>

#include "cruise_control.h"
#include "vesc_protocol.h"

namespace {

int failures = 0;

void expect(bool ok, const char *what) {
  if (!ok) {
    printf("FAIL: %s\n", what);
    failures++;
  }
}

CruiseSettings baseSettings() {
  CruiseSettings s = {};
  s.enabled = true;
  s.holdTimeDeciS = 50;
  s.minSpeedKmh = 5;
  s.maxSpeedKmh = 25;
  s.allowedModes = 0;  // any
  return s;
}

CruiseInputs baseInputs() {
  CruiseInputs in = {};
  in.speedKmh = 20.0F;
  in.rpm = 4000.0F;
  in.rpmPerKmh = 200.0F;  // 20 km/h -> 4000 eRPM
  in.avgInputCurrent = 0.0F;
  in.faultCode = 0;
  in.rideMode = 0;
  in.linkLive = true;
  return in;
}

CruiseRuntime fresh() {
  CruiseRuntime rt = {};
  return rt;
}

void testDisabledNeverSends() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  s.enabled = false;
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  expect(!out.send, "disabled: no command");
  expect(out.state == CRUISE_STATE_OFF, "disabled: stays off");
}

void testEngageBelowMinIsBlocked() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  CruiseInputs in = baseInputs();
  in.speedKmh = 3.0F;  // under the 5 km/h floor
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(!out.send, "too slow: no command");
  expect(out.state == CRUISE_STATE_BLOCKED, "too slow: blocked");
  expect(out.outcome == CRUISE_OUTCOME_BLOCKED_SLOW, "too slow: outcome is BLOCKED_SLOW");
}

void testEngageCapturesTargetAndSends() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  expect(out.send, "engage: sends at once");
  expect(out.lisp != nullptr && strcmp(out.lisp, kCruiseLispEngage) == 0, "engage: uses the engage expression");
  expect(out.state == CRUISE_STATE_ACTIVE, "engage: active");
  expect(out.outcome == CRUISE_OUTCOME_ENGAGED, "engage: outcome is ENGAGED");
  expect(rt.targetKmh == 20, "engage: target is the current speed");
}

void testEngageClampsTargetToMax() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  s.maxSpeedKmh = 15;
  CruiseInputs in = baseInputs();
  in.speedKmh = 22.0F;  // above the ceiling
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(out.state == CRUISE_STATE_ACTIVE, "over max: still engages");
  expect(rt.targetKmh == 15, "over max: target clamped to the ceiling");
}

void testEngageNoGearingIsBlocked() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  CruiseInputs in = baseInputs();
  in.rpmPerKmh = 0.0F;  // wheel size / gearing unset
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(!out.send && out.state == CRUISE_STATE_BLOCKED, "no gearing: blocked");
  expect(out.outcome == CRUISE_OUTCOME_BLOCKED_GEARING, "no gearing: outcome is BLOCKED_GEARING");
}

void testEngageModeBlocked() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  s.allowedModes = (uint8_t)(1u << (CRUISE_MODE_ECO - 1));  // Eco only
  CruiseInputs in = baseInputs();
  in.rideMode = CRUISE_MODE_SPORT;
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(!out.send && out.state == CRUISE_STATE_BLOCKED, "sport while Eco-only: blocked");
  expect(out.outcome == CRUISE_OUTCOME_BLOCKED_MODE, "sport while Eco-only: outcome is BLOCKED_MODE");
}

void testActiveSendsNothingMore() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);  // engages and sends
  // The VESC application holds the setpoint, so there is no heartbeat: a later
  // cycle stays silent while the conditions hold.
  const CruiseOutput later = cruiseControlStep(rt, s, baseInputs(), false, false, 1000 + 500);
  expect(!later.send, "active: no heartbeat frame");
  expect(later.outcome == CRUISE_OUTCOME_NONE, "active: no outcome on a quiet cycle");
  expect(later.state == CRUISE_STATE_ACTIVE, "active: still active");
}

void testFaultReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  CruiseInputs in = baseInputs();
  in.faultCode = 4;  // over-current
  const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1100);
  expect(out.send, "fault: sends a release");
  expect(out.lisp != nullptr && strcmp(out.lisp, kCruiseLispRelease) == 0, "fault: release expression");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_FAULT, "fault: outcome is RELEASED_FAULT");
  expect(out.state == CRUISE_STATE_OFF, "fault: off");
}

void testLinkLossReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  CruiseInputs in = baseInputs();
  in.linkLive = false;
  const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1100);
  expect(out.send && strcmp(out.lisp, kCruiseLispRelease) == 0, "link lost: releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_LINK, "link lost: outcome is RELEASED_LINK");
  expect(out.state == CRUISE_STATE_OFF, "link lost: off");
}

void testDisableReleases() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  s.enabled = false;
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), false, false, 1100);
  expect(out.send && strcmp(out.lisp, kCruiseLispRelease) == 0, "disabled while active: releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_DISABLED, "disabled: outcome is RELEASED_DISABLED");
}

void testDisengageRequestReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), false, true, 1100);
  expect(out.send && strcmp(out.lisp, kCruiseLispRelease) == 0, "cancel: releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_CANCEL, "cancel: outcome is RELEASED_CANCEL");
  expect(out.state == CRUISE_STATE_OFF, "cancel: off");
}

void testSpeedDropReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  CruiseInputs in = baseInputs();
  in.speedKmh = 2.0F;  // braked under the floor
  const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1100);
  expect(out.send && strcmp(out.lisp, kCruiseLispRelease) == 0, "braked below the floor: releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_SPEED, "braked: outcome is RELEASED_SPEED");
}

void testBrakeReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  CruiseInputs in = baseInputs();
  in.avgInputCurrent = -12.0F;  // regenerative braking current
  const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1100);
  expect(out.send && strcmp(out.lisp, kCruiseLispRelease) == 0, "brake: releases");
  expect(out.outcome == CRUISE_OUTCOME_RELEASED_BRAKE, "brake: outcome is RELEASED_BRAKE");
  expect(out.state == CRUISE_STATE_OFF, "brake: off");
  // A light downhill regen stays under the threshold and must not release.
  CruiseRuntime rt2 = fresh();
  cruiseControlStep(rt2, s, baseInputs(), true, false, 1000);
  CruiseInputs coast = baseInputs();
  coast.avgInputCurrent = -1.0F;
  const CruiseOutput held = cruiseControlStep(rt2, s, coast, false, false, 1100);
  expect(!held.send && held.state == CRUISE_STATE_ACTIVE, "light regen: cruise holds");
}

void testModeAllowedWindow() {
  CruiseSettings s = baseSettings();
  s.allowedModes = (uint8_t)(1u << (CRUISE_MODE_DRIVE - 1));
  expect(cruiseModeAllowed(s, CRUISE_MODE_NONE), "unreported mode is never blocked");
  expect(cruiseModeAllowed(s, CRUISE_MODE_DRIVE), "allowed mode passes");
  expect(!cruiseModeAllowed(s, CRUISE_MODE_SPORT), "excluded mode is refused");
  CruiseSettings none = baseSettings();
  none.allowedModes = 0;
  expect(!cruiseModeAllowed(none, CRUISE_MODE_ECO), "empty allow list refuses a reported mode");
  expect(cruiseModeAllowed(none, CRUISE_MODE_NONE), "empty allow list still admits an unreported mode");
}

void testEncodeHelpers() {
  expect(vescEncodeDuty(0.5F) == 50000, "duty x1e5");
  expect(vescEncodeDuty(1.5F) == 100000, "duty clamps to 1");
  expect(vescEncodeDuty(-2.0F) == -100000, "duty clamps to -1");
  expect(vescEncodeCurrent(-12.5F) == -12500, "current x1e3");
}

}  // namespace

int main() {
  testDisabledNeverSends();
  testEngageBelowMinIsBlocked();
  testEngageCapturesTargetAndSends();
  testEngageClampsTargetToMax();
  testEngageNoGearingIsBlocked();
  testEngageModeBlocked();
  testActiveSendsNothingMore();
  testFaultReleases();
  testLinkLossReleases();
  testDisableReleases();
  testDisengageRequestReleases();
  testSpeedDropReleases();
  testBrakeReleases();
  testModeAllowedWindow();
  testEncodeHelpers();
  if (failures == 0) printf("cruise_control: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
