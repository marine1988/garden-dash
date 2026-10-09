// Host tests for the display-side cruise-control state machine.
//
// The module is pure logic, so these drive it directly: no controller, no
// radio, no LVGL. Each case fixes the settings and one cycle of inputs and
// checks what the dashboard would have sent, if anything.

#include <cstdio>

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
  s.deadbandHundredths = 2;
  s.allowedModes = 0;  // any
  return s;
}

CruiseInputs baseInputs() {
  CruiseInputs in = {};
  in.speedKmh = 20.0F;
  in.rpm = 4000.0F;
  in.rpmPerKmh = 200.0F;  // 20 km/h -> 4000 eRPM
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
}

void testEngageCapturesTargetAndSends() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  expect(out.send, "engage: sends at once");
  expect(out.command == VESC_COMM_SET_RPM, "engage: uses SET_RPM");
  expect(out.value == 4000, "engage: eRPM target matches the drivetrain ratio");
  expect(out.state == CRUISE_STATE_ACTIVE, "engage: active");
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
  expect(out.value == 3000, "over max: eRPM follows the clamped target");
}

void testHeartbeatPeriod() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);  // engages and sends

  CruiseOutput early = cruiseControlStep(rt, s, baseInputs(), false, false, 1000 + kCruiseCommandPeriodMs - 1);
  expect(!early.send, "heartbeat: silent inside the period");

  CruiseOutput due = cruiseControlStep(rt, s, baseInputs(), false, false, 1000 + kCruiseCommandPeriodMs);
  expect(due.send && due.command == VESC_COMM_SET_RPM, "heartbeat: resends once due");
}

void testFaultReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  CruiseInputs in = baseInputs();
  in.faultCode = 4;  // over-current
  const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1100);
  expect(out.send, "fault: sends a release");
  expect(out.command == VESC_COMM_SET_CURRENT, "fault: release is SET_CURRENT");
  expect(out.value == kCruiseReleaseValue, "fault: release is zero current");
  expect(out.state == CRUISE_STATE_OFF, "fault: off");
}

void testLinkLossReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  CruiseInputs in = baseInputs();
  in.linkLive = false;
  const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1100);
  expect(out.send && out.value == kCruiseReleaseValue, "link lost: releases");
  expect(out.state == CRUISE_STATE_OFF, "link lost: off");
}

void testDisableReleases() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  s.enabled = false;
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), false, false, 1100);
  expect(out.send && out.value == kCruiseReleaseValue, "disabled while active: releases");
}

void testDisengageRequestReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), false, true, 1100);
  expect(out.send && out.value == kCruiseReleaseValue, "cancel: releases");
  expect(out.state == CRUISE_STATE_OFF, "cancel: off");
}

void testSpeedDropReleases() {
  CruiseRuntime rt = fresh();
  const CruiseSettings s = baseSettings();
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  CruiseInputs in = baseInputs();
  in.speedKmh = 2.0F;  // braked under the floor
  const CruiseOutput out = cruiseControlStep(rt, s, in, false, false, 1100);
  expect(out.send && out.value == kCruiseReleaseValue, "braked below the floor: releases");
}

void testModeNotAllowedBlocks() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  s.allowedModes = (uint8_t)(1u << (CRUISE_MODE_ECO - 1));  // Eco only
  CruiseInputs in = baseInputs();
  in.rideMode = CRUISE_MODE_SPORT;
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(!out.send && out.state == CRUISE_STATE_BLOCKED, "sport while Eco-only: blocked");
}

void testModeAllowedWindow() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  s.allowedModes = (uint8_t)(1u << (CRUISE_MODE_DRIVE - 1));
  CruiseInputs in = baseInputs();
  in.rideMode = CRUISE_MODE_DRIVE;
  const CruiseOutput out = cruiseControlStep(rt, s, in, true, false, 1000);
  expect(out.state == CRUISE_STATE_ACTIVE, "drive allowed: engages");
  expect(cruiseModeAllowed(s, CRUISE_MODE_NONE), "unreported mode is never blocked");
  expect(!cruiseModeAllowed(s, CRUISE_MODE_SPORT), "excluded mode is refused");
  CruiseSettings none = baseSettings();
  none.allowedModes = 0;
  expect(!cruiseModeAllowed(none, CRUISE_MODE_ECO), "empty allow list refuses a reported mode");
  expect(cruiseModeAllowed(none, CRUISE_MODE_NONE), "empty allow list still admits an unreported mode");
}

void testAdjustTarget() {
  CruiseRuntime rt = fresh();
  CruiseSettings s = baseSettings();
  expect(cruiseControlAdjustTarget(rt, s, 3) == 0, "adjust while off is a no-op");
  cruiseControlStep(rt, s, baseInputs(), true, false, 1000);
  expect(cruiseControlAdjustTarget(rt, s, 3) == 23, "adjust raises the target");
  expect(cruiseControlAdjustTarget(rt, s, 100) == 25, "adjust clamps to the ceiling");
  expect(cruiseControlAdjustTarget(rt, s, -100) == 5, "adjust clamps to the floor");
  // The nudge forces the next step to send the new target at once.
  const CruiseOutput out = cruiseControlStep(rt, s, baseInputs(), false, false, 1000 + 1);
  expect(out.send && out.value == 1000, "adjusted target goes out on the next step");
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
  testHeartbeatPeriod();
  testFaultReleases();
  testLinkLossReleases();
  testDisableReleases();
  testDisengageRequestReleases();
  testSpeedDropReleases();
  testModeNotAllowedBlocks();
  testModeAllowedWindow();
  testAdjustTarget();
  testEncodeHelpers();
  if (failures == 0) printf("cruise_control: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
