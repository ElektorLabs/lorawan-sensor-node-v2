#pragma once

#include <Arduino.h>
#include <cstdio>
#include <cstring>

#include "TaggedStream.h"

/*
 * LoraAtHelpers.h
 *
 * Minimal helper for AT-style LoRaWAN modules. Usage:
 *
 *   #include "TaggedStream.h"
 *   #include "LoraAtHelpers.h"
 *
 *   StreamTagState loraTag{"[LoRa] ", true};
 *   LoraAtState lora;
 *
 *   void setup() {
 *     loraSerial.begin(...);
 *     loraInitState(lora, loraSerial, Serial, loraTag);
 *     loraStartJoin(lora, 300000);  // Issue AT+JOIN (retry every 5 minutes)
 *   }
 *
 *   void loop() {
 *     loraPoll(lora);  // Pump UART bytes and watch for +EVT:JOINED, etc.
 *   }
 *
 *   if (loraJoined(lora)) {
 *     loraSendHexPayload(lora, 2, "414243", "OK", 6000);
 *   }
 *
 * The helpers keep the tagged console output readable, detect join events,
 * and provide a simple command wrapper with timeout handling.
 */

// Lightweight state bag for the AT bridge. Designed so sketches can keep a
// single instance and call the inline helpers below.
struct LoraAtState {
  HardwareSerial *serial = nullptr;
  Stream *console = nullptr;
  StreamTagState *tag = nullptr;
  bool commandInProgress = false;
  bool modulePresent = true;
  bool joinAccepted = false;
  bool joined = false;
  unsigned long lastJoinAttemptMs = 0;
  char lineBuffer[160];
  size_t lineLength = 0;
};

inline void loraInitState(LoraAtState &state,
                          HardwareSerial &serial,
                          Stream &console,
                          StreamTagState &tag) {
  state.serial = &serial;
  state.console = &console;
  state.tag = &tag;
  state.commandInProgress = false;
  state.modulePresent = true;
  state.joinAccepted = false;
  state.joined = false;
  state.lineLength = 0;
}

inline void loraProcessLine(LoraAtState &state, const char *line) {
  if (line == nullptr || line[0] == '\0') {
    return;
  }
  if (strstr(line, "+EVT:JOINED")) {
    state.joined = true;
    if (state.console) {
      state.console->println("[LoRa] Event: join confirmed.");
    }
  } else if (strstr(line, "+EVT:TX_DONE")) {
    if (state.console) {
      state.console->println("[LoRa] Event: uplink acknowledged.");
    }
  }
}

inline void loraHandleByte(LoraAtState &state, int byteValue) {
  if (state.console && state.tag) {
    printTaggedByte(*state.console, *state.tag, byteValue);
  }

  if (byteValue == '\n') {
    state.lineBuffer[state.lineLength] = '\0';
    if (state.lineLength > 0) {
      loraProcessLine(state, state.lineBuffer);
    }
    state.lineLength = 0;
  } else if (byteValue != '\r') {
    if (state.lineLength < sizeof(state.lineBuffer) - 1) {
      state.lineBuffer[state.lineLength++] = static_cast<char>(byteValue);
    } else {
      state.lineLength = 0;
    }
  }
}

inline void loraFlushInput(LoraAtState &state) {
  if (!state.serial) {
    return;
  }
  while (state.serial->available()) {
    loraHandleByte(state, state.serial->read());
  }
}

inline void loraPoll(LoraAtState &state) {
  loraFlushInput(state);
}

inline bool loraJoined(const LoraAtState &state) {
  return state.joined;
}

inline bool loraWaitForResponse(LoraAtState &state, const char *expected, uint32_t timeoutMs) {
  if (!state.serial || expected == nullptr || expected[0] == '\0') {
    delay(timeoutMs);
    loraFlushInput(state);
    return true;
  }

  const size_t expectLen = strlen(expected);
  size_t matchIndex = 0;
  const unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (state.serial->available()) {
      const int byteValue = state.serial->read();
      loraHandleByte(state, byteValue);

      if (byteValue == expected[matchIndex]) {
        matchIndex++;
        if (matchIndex == expectLen) {
          return true;
        }
      } else {
        matchIndex = (byteValue == expected[0]) ? 1 : 0;
      }
    }
  }

  return false;
}

inline bool loraSendCommand(LoraAtState &state,
                            const char *expected,
                            uint32_t timeoutMs,
                            const char *command) {
  if (!state.serial || !state.console) {
    return false;
  }

  state.commandInProgress = true;
  loraFlushInput(state);

  state.console->print("[LoRa] >> ");
  state.console->print(command);

  state.serial->print(command);
  const bool ok = loraWaitForResponse(state, expected, timeoutMs);
  state.commandInProgress = false;

  if (!ok && state.console) {
    state.console->print("[LoRa] Command timeout waiting for: ");
    state.console->println(expected ? expected : "(any)");
  }

  return ok;
}

inline bool loraSendHexPayload(LoraAtState &state,
                               int port,
                               const char *hexPayload,
                               const char *expected,
                               uint32_t timeoutMs) {
  char command[384];
  snprintf(command, sizeof(command), "AT+SEND=%d:%s\r\n", port, hexPayload);
  return loraSendCommand(state, expected, timeoutMs, command);
}

inline bool loraStartJoin(LoraAtState &state, unsigned long retryMs) {
  const unsigned long now = millis();
  if (!state.serial) {
    return false;
  }
  if (retryMs > 0 && (now - state.lastJoinAttemptMs) < retryMs) {
    return state.joinAccepted;
  }
  state.lastJoinAttemptMs = now;
  state.console->println("[LoRa] Sending AT+JOIN...");
  state.joinAccepted = loraSendCommand(state, "OK", 1000, "AT+JOIN\r\n");
  if (state.joinAccepted) {
    state.console->println("[LoRa] Join command accepted; awaiting +EVT:JOINED.");
  } else {
    state.console->println("[LoRa] Join start failed.");
  }
  return state.joinAccepted;
}
