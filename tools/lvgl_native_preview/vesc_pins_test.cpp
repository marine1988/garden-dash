// The VESC link's pins.
//
// A wrong pin here is silent: the display boots, draws its dashboards and simply
// never sees the controller, with no error anywhere. So both builds pin the
// numbers, and the USB-header build is this same source compiled a second time
// with GARDEN_VESC_ON_USB_HEADER set.

#include <cstdio>

#include "config.h"
#include "garden_log.h"

namespace {

int failures = 0;

void expect(bool ok, const char *what) {
  if (!ok) {
    printf("FAIL: %s\n", what);
    failures++;
  }
}

}  // namespace

int main() {
#if GARDEN_VESC_ON_USB_HEADER
  // The power header's pair, the same two the on-board USB-serial chip drives.
  expect(VESC_RX_PIN == 3, "usb-header build: the link's RX is GPIO3");
  expect(VESC_TX_PIN == 1, "usb-header build: the link's TX is GPIO1");
#else
  // CN1, whose pins are free GPIOs.
  expect(VESC_RX_PIN == 27, "default build: the link's RX is GPIO27 (CN1)");
  expect(VESC_TX_PIN == 22, "default build: the link's TX is GPIO22 (CN1)");
#endif
  expect(VESC_BAUD == 115200, "the link runs at 115200");

  // GPIO16/17 are hard-wired to the RGB LED's legs and are not broken out, so
  // the link must never be moved onto them.
  expect(VESC_RX_PIN != 16 && VESC_RX_PIN != 17 && VESC_TX_PIN != 16 && VESC_TX_PIN != 17,
         "the link stays off the RGB LED pins");

  // The log has to compile in either build: on the host it is a no-op, in the
  // USB-header build it must not print into the port that carries the link.
  gardenLog("vesc pins test");
  gardenLogf("vesc pins test %d\n", 1);

  if (failures == 0) printf("vesc_pins: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
