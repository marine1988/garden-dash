#include "vesc_profile.h"

#include <math.h>
#include <stdio.h>

int32_t vescProfileErpmForSpeed(uint16_t speedKmh, bool reverse, float rpmPerKmh) {
  if (!isfinite(rpmPerKmh) || rpmPerKmh <= 0.0F) return 0;
  const float erpm = (float)speedKmh * rpmPerKmh;
  return (int32_t)lroundf(reverse ? -erpm : erpm);
}

VescProfileTarget vescProfileTarget(uint16_t forwardSpeedKmh, uint16_t reverseSpeedKmh,
                                    uint16_t currentMaxScalePercent, uint16_t currentMinScalePercent,
                                    float rpmPerKmh) {
  VescProfileTarget target = {};
  target.currentMaxScale = (float)currentMaxScalePercent / 100.0F;
  target.currentMinScale = (float)currentMinScalePercent / 100.0F;
  target.minErpm = (float)vescProfileErpmForSpeed(reverseSpeedKmh, true, rpmPerKmh);
  target.maxErpm = (float)vescProfileErpmForSpeed(forwardSpeedKmh, false, rpmPerKmh);
  return target;
}

size_t vescProfileLispFieldWeakening(uint16_t fieldWeakeningAmps, char *out, size_t cap) {
  if (!out || cap == 0) return 0;
  if (fieldWeakeningAmps > kVescProfileMaxFieldWeakeningAmps) {
    fieldWeakeningAmps = kVescProfileMaxFieldWeakeningAmps;
  }
  const int written =
      snprintf(out, cap, "(conf-set 'foc-fw-current-max %u)", (unsigned)fieldWeakeningAmps);
  if (written < 0 || (size_t)written >= cap) return 0;
  return (size_t)written;
}

bool vescProfileReadBackMatches(const VescProfileTarget &target, const VescMcconfTemp &reply) {
  // The controller stores the scales it accepts in coarse steps and clamps ERPM
  // to its own hardware limits, so the tolerance separates "the write landed"
  // from "the controller rounded it", not sloppiness.
  const float scaleTolerance = 0.02F;  // two percent
  if (fabsf(reply.currentMinScale - target.currentMinScale) > scaleTolerance) return false;
  if (fabsf(reply.currentMaxScale - target.currentMaxScale) > scaleTolerance) return false;

  const float erpmTolerance = fmaxf(20.0F, fabsf(target.maxErpm) * 0.02F);
  if (fabsf(reply.maxErpm - target.maxErpm) > erpmTolerance) return false;
  if (fabsf(reply.minErpm - target.minErpm) > erpmTolerance) return false;
  return true;
}

uint16_t vescProfileSpeedFromErpm(int32_t erpm, float rpmPerKmh) {
  if (!isfinite(rpmPerKmh) || rpmPerKmh <= 0.0F) return 0;
  const float speed = fabsf((float)erpm) / rpmPerKmh;
  if (speed <= 0.0F) return 0;
  if (speed >= 65535.0F) return 65535;
  return (uint16_t)lroundf(speed);
}

uint16_t vescProfileScalePercent(float scale) {
  if (!isfinite(scale) || scale <= 0.0F) return 0;
  float percent = scale * 100.0F;
  if (percent >= 100.0F) return 100;
  return (uint16_t)lroundf(percent);
}
