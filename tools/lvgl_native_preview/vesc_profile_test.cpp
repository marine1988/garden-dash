// Host tests for the controller-profile logic: the unit conversions that reach a
// live motor, and the read-back check that decides whether a write is reported
// as applied. Pure logic, so it runs without a controller.

#include <cmath>
#include <cstdio>
#include <cstring>

#include "vesc_profile.h"

namespace {

int failures = 0;

void expect(bool ok, const char *what) {
  if (!ok) {
    printf("FAIL: %s\n", what);
    failures++;
  }
}

void expectNear(float actual, float wanted, const char *what) {
  if (std::fabs(actual - wanted) > 1e-3F) {
    printf("FAIL: %s (got %.6f, want %.6f)\n", what, actual, wanted);
    failures++;
  }
}

void testErpmForSpeed() {
  // A forward limit is positive and a reverse limit negative, at the same
  // magnitude. 100 eRPM per km/h makes the arithmetic obvious.
  expect(vescProfileErpmForSpeed(25, false, 100.0F) == 2500, "forward limit is positive ERPM");
  expect(vescProfileErpmForSpeed(25, true, 100.0F) == -2500, "reverse limit is negative ERPM");
  expect(vescProfileErpmForSpeed(0, false, 100.0F) == 0, "zero limit is zero ERPM");
  // A factor the display cannot compute must not become a limit on the motor.
  expect(vescProfileErpmForSpeed(25, false, 0.0F) == 0, "zero factor yields zero ERPM");
  expect(vescProfileErpmForSpeed(25, false, -1.0F) == 0, "negative factor yields zero ERPM");
}

void testTarget() {
  const VescProfileTarget target = vescProfileTarget(25, 5, 50, 80, 100.0F);
  expectNear(target.currentMaxScale, 0.5F, "drive scale is percent/100");
  expectNear(target.currentMinScale, 0.8F, "brake scale is percent/100");
  expectNear(target.maxErpm, 2500.0F, "target forward ERPM");
  expectNear(target.minErpm, -500.0F, "target reverse ERPM");
}

void testLispExpression() {
  char buffer[kVescProfileLispMaxLen];
  const size_t n = vescProfileLispFieldWeakening(10, buffer, sizeof(buffer));
  expect(n > 0, "field weakening expression written");
  expect(std::strcmp(buffer, "(conf-set 'foc-fw-current-max 10)") == 0,
         "field weakening expression text");

  // A value past the ceiling is clamped rather than sent.
  vescProfileLispFieldWeakening(1000, buffer, sizeof(buffer));
  char clamped[kVescProfileLispMaxLen];
  std::snprintf(clamped, sizeof(clamped), "(conf-set 'foc-fw-current-max %u)",
                (unsigned)kVescProfileMaxFieldWeakeningAmps);
  expect(std::strcmp(buffer, clamped) == 0, "field weakening clamps to the ceiling");

  expect(vescProfileLispFieldWeakening(10, buffer, 4) == 0, "expression refuses a tiny buffer");
}

void testReadBack() {
  const VescProfileTarget target = vescProfileTarget(25, 5, 50, 80, 100.0F);

  VescMcconfTemp exact = {};
  exact.currentMaxScale = 0.5F;
  exact.currentMinScale = 0.8F;
  exact.maxErpm = 2500.0F;
  exact.minErpm = -500.0F;
  expect(vescProfileReadBackMatches(target, exact), "an exact reply matches");

  // The controller rounds and clamps a little; that must not read as a failure.
  VescMcconfTemp rounded = exact;
  rounded.currentMaxScale = 0.505F;
  rounded.maxErpm = 2510.0F;
  expect(vescProfileReadBackMatches(target, rounded), "a rounded reply still matches");

  // A limit the controller never took is a failure: this is what stops the UI
  // showing a profile as applied when the link dropped it.
  VescMcconfTemp dropped = exact;
  dropped.maxErpm = 0.0F;
  expect(!vescProfileReadBackMatches(target, dropped), "an unapplied limit fails");
  dropped = exact;
  dropped.currentMaxScale = 1.0F;
  expect(!vescProfileReadBackMatches(target, dropped), "an unapplied scale fails");
}

void testInverse() {
  expect(vescProfileSpeedFromErpm(2500, 100.0F) == 25, "ERPM back to km/h");
  expect(vescProfileSpeedFromErpm(-2500, 100.0F) == 25, "reverse ERPM is reported as a magnitude");
  expect(vescProfileSpeedFromErpm(2500, 0.0F) == 0, "no factor yields no speed");

  expect(vescProfileScalePercent(0.5F) == 50, "scale fraction back to percent");
  expect(vescProfileScalePercent(1.0F) == 100, "full scale is 100 percent");
  expect(vescProfileScalePercent(0.0F) == 0, "zero scale");
}

}  // namespace

int main() {
  testErpmForSpeed();
  testTarget();
  testLispExpression();
  testReadBack();
  testInverse();
  if (failures == 0) printf("vesc_profile: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
