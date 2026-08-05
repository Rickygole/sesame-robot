#include "mailbox.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace sesame {

namespace {
// Guards the state snapshot. Held for a struct copy only -- never across
// I/O, parsing, or anything that can block.
portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;
RobotState g_state;
}  // namespace

bool Mailbox::begin(uint8_t depth) {
  queue_ = xQueueCreate(depth, sizeof(core::Command));
  return queue_ != nullptr;
}

bool Mailbox::postCommand(const core::Command& cmd) {
  if (queue_ == nullptr) {
    return false;
  }
  // Zero timeout: the network task must never block waiting on the
  // motion loop. A full queue is reported to the client as a rejection.
  return xQueueSend(static_cast<QueueHandle_t>(queue_), &cmd, 0) == pdTRUE;
}

bool Mailbox::takeCommand(core::Command* out) {
  if (queue_ == nullptr || out == nullptr) {
    return false;
  }
  return xQueueReceive(static_cast<QueueHandle_t>(queue_), out, 0) == pdTRUE;
}

void Mailbox::publish(const RobotState& s) {
  portENTER_CRITICAL(&g_stateMux);
  g_state = s;
  portEXIT_CRITICAL(&g_stateMux);
}

RobotState Mailbox::state() const {
  RobotState copy;
  portENTER_CRITICAL(&g_stateMux);
  copy = g_state;
  portEXIT_CRITICAL(&g_stateMux);
  return copy;
}

}  // namespace sesame
