// Host tests for the ride-profile logic: the conversions that reach a live motor,
// the single LispBM expression that carries a profile, and the read-back check
// that decides whether a write is reported as applied. Pure logic, so it runs
// without a controller.

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
  const VescProfileTarget target = vescProfileTarget(25, 5, 50, 80, 30, 100.0F);
  expectNear(target.currentMaxScale, 0.5F, "drive scale is percent/100");
  expectNear(target.currentMinScale, 0.8F, "brake scale is percent/100");
  expectNear(target.maxErpm, 2500.0F, "target forward ERPM");
  expectNear(target.minErpm, -500.0F, "target reverse ERPM");
  expectNear(target.inCurrentMax, 30.0F, "battery limit is carried for the read-back");
}

void testLispExpression() {
  char buffer[kVescProfileLispMaxLen];
  const size_t n = vescProfileLisp(25, 5, 60, 80, 10, 120, 30, 100.0F, buffer, sizeof(buffer));
  expect(n > 0, "profile expression written");
  // Every field goes out in one command, because the firmware's REPL takes one
  // command per half second and a profile must land as a unit.
  expect(std::strstr(buffer, "(conf-set 'l-max-erpm 2500)") != nullptr, "forward speed limit");
  expect(std::strstr(buffer, "(conf-set 'l-min-erpm -500)") != nullptr, "reverse speed limit");
  expect(std::strstr(buffer, "(conf-set 'l-current-max-scale 0.60)") != nullptr, "accel scale");
  expect(std::strstr(buffer, "(conf-set 'l-current-min-scale 0.80)") != nullptr, "brake scale");
  expect(std::strstr(buffer, "(conf-set 'foc-fw-current-max 10)") != nullptr, "field weakening");
  expect(std::strstr(buffer, "(conf-set 'l-current-max 120)") != nullptr, "motor current limit");
  expect(std::strstr(buffer, "(conf-set 'l-in-current-max 30)") != nullptr, "battery current limit");
  expect(buffer[0] == '{' && buffer[n - 1] == '}', "the forms are wrapped in one block");

  // A factor the display cannot compute leaves the speed limits alone rather than
  // sending zero, which would clamp the motor to a standstill.
  const size_t noSpeed = vescProfileLisp(25, 5, 60, 80, 10, 120, 30, 0.0F, buffer, sizeof(buffer));
  expect(noSpeed > 0, "expression without a speed factor is still written");
  expect(std::strstr(buffer, "l-max-erpm") == nullptr, "no speed limit without a factor");
  expect(std::strstr(buffer, "l-in-current-max") != nullptr, "the currents are still written");

  // A field weakening typo is clamped rather than sent to a live motor.
  vescProfileLisp(25, 5, 60, 80, 1000, 120, 30, 100.0F, buffer, sizeof(buffer));
  char clamped[64];
  std::snprintf(clamped, sizeof(clamped), "(conf-set 'foc-fw-current-max %u)",
                (unsigned)kVescProfileMaxFieldWeakeningAmps);
  expect(std::strstr(buffer, clamped) != nullptr, "field weakening clamps to the ceiling");

  expect(vescProfileLisp(25, 5, 60, 80, 10, 120, 30, 100.0F, buffer, 8) == 0,
         "expression refuses a buffer it cannot fill");

  // The whole expression has to fit a VESC short packet's 255-byte payload
  // alongside the command id and the trailing NUL, even at the largest values a
  // rider can set. This is the check that catches a field being added later.
  const size_t longest = vescProfileLisp(200, 100, 100, 100, kVescProfileMaxFieldWeakeningAmps,
                                         500, 500, 10000.0F, buffer, sizeof(buffer));
  expect(longest > 0 && longest + 2 <= 255, "the longest profile fits one packet");
}

void testReadBack() {
  const VescProfileTarget target = vescProfileTarget(25, 5, 50, 80, 30, 100.0F);

  VescMcconfTemp exact = {};
  exact.currentMaxScale = 0.5F;
  exact.currentMinScale = 0.8F;
  exact.maxErpm = 2500.0F;
  exact.minErpm = -500.0F;
  exact.inCurrentMax = 30.0F;
  expect(vescProfileReadBackMatches(target, exact), "an exact reply matches");

  // The controller rounds and clamps a little; that must not read as a failure.
  VescMcconfTemp rounded = exact;
  rounded.currentMaxScale = 0.505F;
  rounded.maxErpm = 2510.0F;
  rounded.inCurrentMax = 30.4F;
  expect(vescProfileReadBackMatches(target, rounded), "a rounded reply still matches");

  // A limit the controller never took is a failure: this is what stops the UI
  // showing a profile as applied when the link dropped it.
  VescMcconfTemp dropped = exact;
  dropped.maxErpm = 0.0F;
  expect(!vescProfileReadBackMatches(target, dropped), "an unapplied speed limit fails");
  dropped = exact;
  dropped.currentMaxScale = 1.0F;
  expect(!vescProfileReadBackMatches(target, dropped), "an unapplied scale fails");
  dropped = exact;
  dropped.inCurrentMax = 80.0F;
  expect(!vescProfileReadBackMatches(target, dropped), "an unapplied battery limit fails");

  // A field the profile left alone is not compared: the display could not convert
  // the speed, so the controller keeping its own limit is the correct outcome.
  const VescProfileTarget noSpeed = vescProfileTarget(25, 5, 50, 80, 30, 0.0F);
  VescMcconfTemp untouched = exact;
  untouched.maxErpm = 99999.0F;
  untouched.minErpm = -99999.0F;
  expect(vescProfileReadBackMatches(noSpeed, untouched), "an unconverted speed limit is not compared");
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
