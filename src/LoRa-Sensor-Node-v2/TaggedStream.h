/*
 * TaggedStream.h
 *
 * Example:
 *   StreamTagState gpsTag{"[GPS ] ", true};
 *   void loop() {
 *     if (Serial.available()) {
 *       printTaggedByte(Serial, gpsTag, Serial.read());
 *     }
 *   }
 */

#pragma once

#include <Arduino.h>

// Simple helper that keeps track of whether the next character should be
// preceded by a module tag (e.g., "[GPS ] " or "[LoRa] ").
struct StreamTagState {
  const char *label;
  bool atLineStart;
};

inline void printTaggedByte(Stream &target, StreamTagState &state, int byteValue) {
  if (state.atLineStart) {
    target.print(state.label);
    state.atLineStart = false;
  }
  target.write(byteValue);
  if (byteValue == '\n') {
    state.atLineStart = true;
  }
}
