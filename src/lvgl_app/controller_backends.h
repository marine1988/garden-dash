#pragma once

#include "controller_manager.h"

void vescUartBackendBegin();
void vescBleBackendBegin();
void vescBackendStop();
ControllerPollResult vescBackendPoll(ControllerSample &out);

// Cruise control, driven from the telemetry task because it owns the UART/BLE
// transport. The UI only leaves a request; the task applies it on its next poll
// and publishes the resulting state and outcome back for the UI to read.
// A request is consumed once, so calling engage twice engages once.
//
// Only the VESC backends implement these; the FarDriver link has no equivalent
// command, and controllerCapabilities().supportsCruiseControl is false there.
void vescCruiseRequestEngage();
void vescCruiseRequestDisengage();
// False when cruise is idle. `targetKmh` is written only while active.
bool vescCruiseActive(uint16_t &targetKmh);
// The last outcome the state machine produced, and the state it left cruise in.
// Reported as an edge: the UI compares against the value it saw before, so a
// message is shown once per transition. Writes both values under one lock.
void vescCruiseOutcome(CruiseOutcome &outcome, CruiseState &state);

// Profile writes, driven from the telemetry task for the same reason. The UI
// leaves the slot number; the task reads the controller's current limits,
// writes the profile's speed and current limits (store=0, so nothing reaches the
// controller's flash), sets field weakening over LispBM, then reads the limits
// back and reports whether they match. The UI polls vescProfileWriteReportGet().
void vescProfileRequestWrite(uint8_t index);
VescProfileWriteReport vescProfileWriteReportGet();

void farDriverBackendBegin();
void farDriverBackendStop();
ControllerPollResult farDriverBackendPoll(ControllerSample &out);

