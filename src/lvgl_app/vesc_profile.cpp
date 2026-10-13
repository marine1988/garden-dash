#include "vesc_profile.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

int32_t vescProfileErpmForSpeed(uint16_t speedKmh, bool reverse, float rpmPerKmh) {
  if (!isfinite(rpmPerKmh) || rpmPerKmh <= 0.0F) return 0;
  const float erpm = (float)speedKmh * rpmPerKmh;
  return (int32_t)lroundf(reverse ? -erpm : erpm);
}

size_t vescProfileLisp(uint16_t maxSpeedKmh, uint16_t reverseSpeedKmh, uint16_t accelPercent,
                       uint16_t brakePercent, uint16_t fieldWeakeningAmps, uint16_t motorAmps,
                       uint16_t batteryAmps, float rpmPerKmh, char *out, size_t cap) {
  if (!out || cap == 0) return 0;
  if (fieldWeakeningAmps > kVescProfileMaxFieldWeakeningAmps) {
    fieldWeakeningAmps = kVescProfileMaxFieldWeakeningAmps;
  }

  // Appends one form, refusing rather than truncating: a half-written expression
  // would be evaluated by the controller as something the rider never asked for.
  // Each form is formatted into a local buffer first, so the appender itself
  // needs no variadic machinery.
  size_t used = 0;
  const auto append = [&](const char *text) -> bool {
    const size_t len = strlen(text);
    if (used + len + 1 > cap) return false;
    memcpy(out + used, text, len);
    used += len;
    out[used] = '\0';
    return true;
  };
  char form[64];

  if (!append("{")) return 0;
  // A factor of zero means the display cannot convert km/h to ERPM (no wheel
  // size, no pole pairs). Sending a zero limit would clamp the motor to a
  // standstill, so those two fields are left exactly as the controller has them.
  if (isfinite(rpmPerKmh) && rpmPerKmh > 0.0F) {
    snprintf(form, sizeof(form), "(conf-set 'l-max-erpm %ld)",
             (long)vescProfileErpmForSpeed(maxSpeedKmh, false, rpmPerKmh));
    if (!append(form)) return 0;
    snprintf(form, sizeof(form), "(conf-set 'l-min-erpm %ld)",
             (long)vescProfileErpmForSpeed(reverseSpeedKmh, true, rpmPerKmh));
    if (!append(form)) return 0;
  }
  snprintf(form, sizeof(form), "(conf-set 'l-current-max-scale %.2f)", (double)accelPercent / 100.0);
  if (!append(form)) return 0;
  snprintf(form, sizeof(form), "(conf-set 'l-current-min-scale %.2f)", (double)brakePercent / 100.0);
  if (!append(form)) return 0;
  snprintf(form, sizeof(form), "(conf-set 'foc-fw-current-max %u)", (unsigned)fieldWeakeningAmps);
  if (!append(form)) return 0;
  snprintf(form, sizeof(form), "(conf-set 'l-current-max %u)", (unsigned)motorAmps);
  if (!append(form)) return 0;
  snprintf(form, sizeof(form), "(conf-set 'l-in-current-max %u)", (unsigned)batteryAmps);
  if (!append(form)) return 0;
  if (!append("}")) return 0;
  return used;
}

VescProfileTarget vescProfileTarget(uint16_t maxSpeedKmh, uint16_t reverseSpeedKmh,
                                    uint16_t accelPercent, uint16_t brakePercent,
                                    uint16_t batteryAmps, float rpmPerKmh) {
  VescProfileTarget target = {};
  target.currentMaxScale = (float)accelPercent / 100.0F;
  target.currentMinScale = (float)brakePercent / 100.0F;
  target.minErpm = (float)vescProfileErpmForSpeed(reverseSpeedKmh, true, rpmPerKmh);
  target.maxErpm = (float)vescProfileErpmForSpeed(maxSpeedKmh, false, rpmPerKmh);
  target.inCurrentMax = (float)batteryAmps;
  return target;
}

bool vescProfileReadBackMatches(const VescProfileTarget &target, const VescMcconfTemp &reply) {
  // The controller stores the scales it accepts in coarse steps and clamps ERPM
  // to its own hardware limits, so the tolerance separates "the write landed"
  // from "the controller rounded it", not sloppiness.
  const float scaleTolerance = 0.02F;  // two percent
  if (fabsf(reply.currentMinScale - target.currentMinScale) > scaleTolerance) return false;
  if (fabsf(reply.currentMaxScale - target.currentMaxScale) > scaleTolerance) return false;

  // A zero target is a field the profile left alone, so there is nothing to
  // compare it against.
  if (target.maxErpm != 0.0F || target.minErpm != 0.0F) {
    const float erpmTolerance = fmaxf(20.0F, fabsf(target.maxErpm) * 0.02F);
    if (target.maxErpm != 0.0F && fabsf(reply.maxErpm - target.maxErpm) > erpmTolerance) return false;
    if (target.minErpm != 0.0F && fabsf(reply.minErpm - target.minErpm) > erpmTolerance) return false;
  }

  const float currentTolerance = fmaxf(1.0F, fabsf(target.inCurrentMax) * 0.02F);
  if (fabsf(reply.inCurrentMax - target.inCurrentMax) > currentTolerance) return false;
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
