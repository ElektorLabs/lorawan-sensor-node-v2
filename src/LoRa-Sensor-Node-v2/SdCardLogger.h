/*
 * SdCardLogger.h
 *
 * Simple helper for logging GPS summaries to the SPI-connected SD card.
 * Pin assignments come from config.h; SPI pins default to MOSI=11, MISO=13,
 * SCK=12, and CS=8 on the ESP32-S3 module.
 *
 * Example:
 *   SdCardLoggerState sd;
 *   constexpr SdCardLoggerConfig cfg{8, 12, 13, 11, "/gps-log.csv"};
 *   void setup() {
 *     Serial.begin(115200);
 *     sdLoggerBegin(sd, cfg, Serial);
 *   }
 *   void loop() {
 *     extern const EbyteGpsStatus &gpsStatus;
 *     sdLoggerLogCsv(sd, gpsStatus);
 *   }
 */

#pragma once

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <cstdio>

#include "EbyteGpsParser.h"

struct SdCardLoggerConfig {
  uint8_t csPin;
  uint8_t sckPin;
  uint8_t misoPin;
  uint8_t mosiPin;
  const char *logFilename;
};

struct SdCardLoggerState {
  SdCardLoggerConfig config{};
  bool initialized = false;
  File logFile;
};

inline bool sdLoggerBegin(SdCardLoggerState &state,
                          const SdCardLoggerConfig &config,
                          Print &logOutput,
                          uint32_t spiFrequencyHz = 8000000) {
  state.initialized = false;
  if (state.logFile) {
    state.logFile.close();
  }

  state.config = config;

  SPI.begin(config.sckPin, config.misoPin, config.mosiPin, config.csPin);
  if (!SD.begin(config.csPin, SPI, spiFrequencyHz)) {
    logOutput.println("[SD  ] SD.begin() failed; logging disabled.");
    return false;
  }

  state.logFile = SD.open(config.logFilename, FILE_APPEND);
  if (!state.logFile) {
    logOutput.print("[SD  ] Failed to open ");
    logOutput.print(config.logFilename);
    logOutput.println(" for append.");
    return false;
  }

  if (state.logFile.size() == 0) {
    state.logFile.println("#UTC_DATE,UTC_TIME,FIX_OK,SATS,LAT,LON,ALT_M,SPEED_KMH");
    state.logFile.flush();
  }

  state.initialized = true;
  logOutput.print("[SD  ] Logging to ");
  logOutput.println(config.logFilename);
  return true;
}

inline void sdLoggerLogLine(SdCardLoggerState &state, const char *line) {
  if (!state.initialized || !state.logFile) {
    return;
  }
  state.logFile.println(line);
  state.logFile.flush();
}

inline void sdLoggerLogCsv(SdCardLoggerState &state,
                           const EbyteGpsStatus &status) {
  if (!state.initialized || !status.hasSentence) {
    return;
  }

  char buffer[160];
  snprintf(buffer,
           sizeof(buffer),
           "%s,%s,%s,%u,%.5f,%.5f,%.1f,%.2f",
           status.dateUtc,
           status.timeUtc,
           status.fixValid ? "1" : "0",
           status.satelliteCount,
           status.latitudeDeg,
           status.longitudeDeg,
           status.altitudeMeters,
           status.speedKmh);
  sdLoggerLogLine(state, buffer);
}
