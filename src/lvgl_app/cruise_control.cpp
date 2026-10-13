#include "cruise_control.h"

#include <math.h>

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
  (void)nowMs;
  CruiseOutput out = {};
  out.state = rt.state;
  out.outcome = CRUISE_OUTCOME_NONE;

  const bool wasActive = rt.state == CRUISE_STATE_ACTIVE;

  // Hands a live cruise back explicitly: the override is cleared and the ADC
  // application takes the throttle over again, instead of the controller
  // holding the setpoint until its own command timeout expires.
  const auto release = [&](CruiseOutcome why) {
    out.send = true;
    out.lisp = kCruiseLispRelease;
    rt.state = CRUISE_STATE_OFF;
    rt.targetKmh = 0;
    out.state = rt.state;
    out.outcome = why;
  };

  // A fault, a dead link, a disabled setting or the rider's own cancel always
  // wins, over a fresh engage request just as much as over a running cruise.
  if (!settings.enabled || requestDisengage || !in.linkLive || in.faultCode != 0) {
    if (wasActive) {
      release(!settings.enabled ? CRUISE_OUTCOME_RELEASED_DISABLED
              : requestDisengage ? CRUISE_OUTCOME_RELEASED_CANCEL
              : !in.linkLive    ? CRUISE_OUTCOME_RELEASED_LINK
                                : CRUISE_OUTCOME_RELEASED_FAULT);
    } else {
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
      out.send = true;
      out.lisp = kCruiseLispEngage;
      out.state = rt.state;
      out.outcome = CRUISE_OUTCOME_ENGAGED;
      return out;
    }
    rt.state = CRUISE_STATE_BLOCKED;
    out.state = rt.state;
    out.outcome = !fastEnough ? CRUISE_OUTCOME_BLOCKED_SLOW
                  : !modeOk   ? CRUISE_OUTCOME_BLOCKED_MODE
                              : CRUISE_OUTCOME_BLOCKED_GEARING;
    return out;
  }

  if (rt.state != CRUISE_STATE_ACTIVE) {
    out.state = rt.state;
    return out;
  }

  // The rider is on the brakes: a regenerative (negative battery-side) current
  // while cruising means braking, so give the motor back at once rather than
  // fighting the brake with the speed loop.
  if (in.avgInputCurrent < kCruiseBrakeRegenAmps) {
    release(CRUISE_OUTCOME_RELEASED_BRAKE);
    return out;
  }

  // The rider has taken over: speed fell back under the window, which is what
  // a stop or a hill start looks like. Give the motor back.
  if (in.speedKmh < (float)settings.minSpeedKmh) {
    release(CRUISE_OUTCOME_RELEASED_SPEED);
    return out;
  }

  // Nothing to send: the ADC application holds the setpoint every cycle, so
  // there is no heartbeat to keep alive.
  out.state = rt.state;
  return out;
}
