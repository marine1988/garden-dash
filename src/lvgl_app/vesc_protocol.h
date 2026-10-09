#pragma once

// VESC UART wire protocol: framing, CRC, and the payload parsers this firmware
// needs. Deliberately free of Arduino and Stream dependencies so the same code
// compiles for the device and for the host parity/regression tests.
//
// Frame layout (payloads <= 255 bytes, the only form a VESC sends for the
// commands used here):
//
//   0x02 | length | payload[length] | crc_hi | crc_lo | 0x03
//
// The CRC is CRC-16/XMODEM over the payload only. Payload byte 0 is the
// command id; every parser below indexes from 1 for that reason.

#include <stddef.h>
#include <stdint.h>

enum : uint8_t {
  VESC_COMM_FW_VERSION = 0,
  VESC_COMM_GET_VALUES = 4,
  // Control commands. Each carries one signed 32-bit argument and is not
  // answered: the controller arms a command timeout as it accepts them and
  // releases the motor if they stop arriving, so a caller sends them on a
  // heartbeat rather than once.
  VESC_COMM_SET_DUTY = 5,
  VESC_COMM_SET_CURRENT = 6,
  VESC_COMM_SET_CURRENT_BRAKE = 7,
  VESC_COMM_SET_RPM = 8,
  VESC_COMM_FORWARD_CAN = 34,
  VESC_COMM_GET_VALUES_SETUP = 47,
};

// Longest frame this module builds: CAN-forward prefix, one command id, and the
// four-byte argument a SET_* command carries.
enum : size_t { VESC_MAX_COMMAND_FRAME = 16 };

// The COMM_GET_VALUES fields this firmware consumes. The packet carries more
// (avg_id, avg_iq, pid position, controller id); they are skipped
// in place rather than stored.
struct VescValues {
  float tempMosfet;
  float tempMotor;
  float avgMotorCurrent;
  float avgInputCurrent;
  float dutyCycle;  // signed modulation fraction, UART int16 / 1000
  float rpm;
  float inpVoltage;
  float ampHours;
  float ampHoursCharged;
  float wattHours;
  float wattHoursCharged;
  int32_t tachometerAbs;
  uint8_t faultCode;
};

// The COMM_GET_VALUES_SETUP fields used when the controller reports its own
// speed and distance, so wheel size and pole pairs are not needed.
struct VescSetupValues {
  float speedMs;
  float distanceAbsM;
  float voltage;
  float tempFet;
};

uint16_t vescCrc16(const uint8_t *buf, size_t len);

// Fundamental phase-to-neutral peak (amplitude) estimate for sinusoidal VESC FOC
// with foc_overmod_factor = 1. Not a measured phase voltage or switching waveform.
float vescEstimatePhasePeakVoltage(float batteryVoltage, float dutyCycle);

// Builds the payload for a command, optionally wrapped in a CAN forward.
// Returns the payload length, or 0 if `cap` is too small.
size_t vescBuildCommandPayload(uint8_t command, uint8_t canId, uint8_t *out, size_t cap);

// Builds the payload for a SET_* control command, which carries one signed
// 32-bit argument (duty x1e5, current in milli-amps, or eRPM), optionally
// wrapped in a CAN forward. Returns the payload length, or 0 if `cap` is too
// small. Deliberately separate from vescBuildCommandPayload: a control command
// takes an argument and gets no reply, so callers send the frame and move on
// instead of waiting for a response the controller never sends.
size_t vescBuildControlPayload(uint8_t command, uint8_t canId, int32_t value, uint8_t *out, size_t cap);

// The scaling the VESC firmware applies to each control argument (its own
// commands.c: duty / 1e5, current / 1e3 A). eRPM is sent unscaled. Both clamp
// their input to the command's range and return 0 for a non-finite value, so a
// caller cannot send a NaN as a motor current.
constexpr float VESC_DUTY_SCALE = 1.0e5F;
constexpr float VESC_CURRENT_SCALE = 1.0e3F;
int32_t vescEncodeDuty(float duty);
int32_t vescEncodeCurrent(float amps);

// Wraps a payload in start/length/CRC/stop. Returns the frame length, or 0 if
// the payload does not fit `cap`.
size_t vescBuildFrame(const uint8_t *payload, uint8_t payloadLen, uint8_t *out, size_t cap);

// Parsers. Each validates the command id and a minimum length before reading,
// and leaves `out` untouched when it returns false.
bool vescParseValues(const uint8_t *payload, size_t len, VescValues &out);
bool vescParseSetupValues(const uint8_t *payload, size_t len, VescSetupValues &out);
bool vescParseFwVersion(const uint8_t *payload, size_t len, uint8_t &major, uint8_t &minor);
