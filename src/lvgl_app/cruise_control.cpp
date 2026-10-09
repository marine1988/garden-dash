#include "cruise_control.h"

#include <math.h>

#include "vesc_protocol.h"

bool cruiseModeAllowed(const CruiseSettings &settings, uint8_t mode) {
  // A controller that reports no mode (VESC standard telemetry answers 0) must
  // not be locked out, or cruise would refuse to engage on the most common
  // link. Only a reported mode the rider excluded — or an empty allow list — is
  // refused.
  if (mode < CRUISE_MODE_ECO || mode > CRUISE_MODE_SPORT) return true;
  return (settings.allowedModes & (uint8_t)(1u << (mode - 1))) != 0;
}

CruiseOutput cruiseControlStep(CruiseRuntime &rt, const CruiseSettings &settings, const CruiseInputs &in,
                               bool requestEngage, bool requestDisengage, uint32_t nowMs) {
  CruiseOutput out = {};
  out.state = rt.state;

  // Hands a live cruise back explicitly: the controller coasts with no torque
  // instead of holding its setpoint until its own command timeout expires.
  const auto release = [&]() {
    out.send = true;
    out.command = VESC_COMM_SET_CURRENT;
    out.value = kCruiseReleaseValue;
    rt.state = CRUISE_STATE_OFF;
    rt.targetKmh = 0;
    rt.targetRpm = 0;
    out.state = rt.state;
  };

  const bool wasActive = rt.state == CRUISE_STATE_ACTIVE;

  // A fault, a dead link or the rider's own cancel always wins, over a fresh
  // engage request just as much as over a running cruise.
  if (!settings.enabled || requestDisengage || !in.linkLive || in.faultCode != 0) {
    if (wasActive) release();
    else {
      rt.state = CRUISE_STATE_OFF;
      out.state = rt.state;
    }
    return out;
  }

  if (requestEngage) {
    const bool fastEnough = in.speedKmh >= (float)settings.minSpeedKmh;
    const bool modeOk = cruiseModeAllowed(settings, in.rideMode);
    const bool convertible = in.rpmPerKmh > 0.0F;
    if (fastEnough && modeOk && convertible) {
      float target = in.speedKmh;
      if (target > (float)settings.maxSpeedKmh) target = (float)settings.maxSpeedKmh;
      if (target < (float)settings.minSpeedKmh) target = (float)settings.minSpeedKmh;
      rt.state = CRUISE_STATE_ACTIVE;
      rt.targetKmh = (uint16_t)lroundf(target);
      rt.rpmPerKmh = in.rpmPerKmh;
      rt.targetRpm = (int32_t)lroundf((float)rt.targetKmh * in.rpmPerKmh);
      rt.lastCommandMs = 0;  // force an immediate send below
    } else {
      rt.state = CRUISE_STATE_BLOCKED;
      out.state = rt.state;
      return out;
    }
  }

  if (rt.state != CRUISE_STATE_ACTIVE) {
    out.state = rt.state;
    return out;
  }

  // The rider has taken over: speed fell back under the window, which is what
  // braking, a stop or a hill start looks like. Give the motor back.
  if (in.speedKmh < (float)settings.minSpeedKmh) {
    release();
    return out;
  }

  // Heartbeat. Re-sent even when nothing changed, because the controller
  // releases the motor the moment these stop arriving.
  if (rt.lastCommandMs == 0 || (uint32_t)(nowMs - rt.lastCommandMs) >= kCruiseCommandPeriodMs) {
    out.send = true;
    out.command = VESC_COMM_SET_RPM;
    out.value = rt.targetRpm;
    rt.lastCommandMs = nowMs;
  }
  out.state = rt.state;
  return out;
}

uint16_t cruiseControlAdjustTarget(CruiseRuntime &rt, const CruiseSettings &settings, int deltaKmh) {
  if (rt.state != CRUISE_STATE_ACTIVE) return rt.targetKmh;
  int target = (int)rt.targetKmh + deltaKmh;
  if (target < (int)settings.minSpeedKmh) target = settings.minSpeedKmh;
  if (target > (int)settings.maxSpeedKmh) target = settings.maxSpeedKmh;
  rt.targetKmh = (uint16_t)target;
  if (rt.rpmPerKmh > 0.0F) rt.targetRpm = (int32_t)lroundf((float)rt.targetKmh * rt.rpmPerKmh);
  rt.lastCommandMs = 0;  // the new target goes out on the next step
  return rt.targetKmh;
}
