/*
 * Example:
 *   EbyteGpsParser parser;
 *   void loop() {
 *     static char sentence[128];
 *     if (readNmeaSentence(sentence)) {
 *       parser.processSentence(sentence);
 *       const EbyteGpsStatus &status = parser.getStatus();
 *       Serial.println(status.latitudeDeg, 6);
 *     }
 *   }
 */

#pragma once

#include <Arduino.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>

struct EbyteGpsStatus {
  bool hasSentence = false;
  bool fixValid = false;
  int satelliteCount = 0;
  unsigned long lastUpdateMs = 0;
  float latitudeDeg = 0.0f;
  float longitudeDeg = 0.0f;
  float altitudeMeters = 0.0f;
  float speedKmh = 0.0f;
  char timeUtc[16] = "--:--:--";
  char dateUtc[16] = "--/--/--";
};

class EbyteGpsParser {
 public:
  const EbyteGpsStatus &getStatus() const {
    return status_;
  }

  bool processSentence(const char *sentence) {
    if (sentence == nullptr || sentence[0] == '\0') {
      return false;
    }

    if (startsWith(sentence, "$GNGGA") || startsWith(sentence, "$GPGGA")) {
      return handleGga(sentence);
    }

    if (startsWith(sentence, "$GNRMC") || startsWith(sentence, "$GPRMC")) {
      return handleRmc(sentence);
    }

    return false;
  }

 private:
  static bool startsWith(const char *text, const char *prefix) {
    return strncmp(text, prefix, strlen(prefix)) == 0;
  }

  static double parseCoordinate(const char *value, const char *direction) {
    if (value == nullptr || direction == nullptr || value[0] == '\0' || direction[0] == '\0') {
      return 0.0;
    }
    const double raw = atof(value);
    const int degrees = static_cast<int>(raw / 100.0);
    const double minutes = raw - (degrees * 100.0);
    double decimalDegrees = degrees + (minutes / 60.0);
    if (direction[0] == 'S' || direction[0] == 'W') {
      decimalDegrees = -decimalDegrees;
    }
    return decimalDegrees;
  }

  static void formatUtcTime(const char *hhmmss, char *output, size_t length) {
    if (hhmmss == nullptr || strlen(hhmmss) < 6) {
      strncpy(output, "--:--:--", length);
      output[length - 1] = '\0';
      return;
    }
    const int hour = (hhmmss[0] - '0') * 10 + (hhmmss[1] - '0');
    const int minute = (hhmmss[2] - '0') * 10 + (hhmmss[3] - '0');
    const int second = (hhmmss[4] - '0') * 10 + (hhmmss[5] - '0');
    snprintf(output, length, "%02d:%02d:%02d", hour, minute, second);
  }

  static void formatUtcDate(const char *ddmmyy, char *output, size_t length) {
    if (ddmmyy == nullptr || strlen(ddmmyy) < 6) {
      strncpy(output, "--/--/--", length);
      output[length - 1] = '\0';
      return;
    }
    const int day = (ddmmyy[0] - '0') * 10 + (ddmmyy[1] - '0');
    const int month = (ddmmyy[2] - '0') * 10 + (ddmmyy[3] - '0');
    const int year = (ddmmyy[4] - '0') * 10 + (ddmmyy[5] - '0');
    snprintf(output, length, "%02d/%02d/%02d", day, month, year);
  }

  bool handleGga(const char *sentence) {
    constexpr size_t kBufferSize = 160;
    char copy[kBufferSize];
    strncpy(copy, sentence, kBufferSize);
    copy[kBufferSize - 1] = '\0';

    char *token = strtok(copy, ",");
    int fieldIndex = 0;
    int fixQuality = 0;
    int satellites = 0;
    char latValue[16] = "";
    char latDir[3] = "";
    char lonValue[16] = "";
    char lonDir[3] = "";
    char altitudeValue[16] = "";

    while (token != nullptr) {
      if (fieldIndex == 2) {
        strncpy(latValue, token, sizeof(latValue) - 1);
      } else if (fieldIndex == 3) {
        strncpy(latDir, token, sizeof(latDir) - 1);
      } else if (fieldIndex == 4) {
        strncpy(lonValue, token, sizeof(lonValue) - 1);
      } else if (fieldIndex == 5) {
        strncpy(lonDir, token, sizeof(lonDir) - 1);
      } else if (fieldIndex == 6) {
        fixQuality = atoi(token);
      } else if (fieldIndex == 7) {
        satellites = atoi(token);
      } else if (fieldIndex == 9) {
        strncpy(altitudeValue, token, sizeof(altitudeValue) - 1);
      }
      token = strtok(nullptr, ",");
      fieldIndex++;
    }

    status_.hasSentence = true;
    status_.fixValid = fixQuality > 0;
    status_.satelliteCount = satellites;
    status_.lastUpdateMs = millis();

    if (latValue[0] && latDir[0]) {
      status_.latitudeDeg = static_cast<float>(parseCoordinate(latValue, latDir));
    }

    if (lonValue[0] && lonDir[0]) {
      status_.longitudeDeg = static_cast<float>(parseCoordinate(lonValue, lonDir));
    }

    if (altitudeValue[0]) {
      status_.altitudeMeters = atof(altitudeValue);
    }

    return true;
  }

  bool handleRmc(const char *sentence) {
    constexpr size_t kBufferSize = 160;
    char copy[kBufferSize];
    strncpy(copy, sentence, kBufferSize);
    copy[kBufferSize - 1] = '\0';

    char *token = strtok(copy, ",");
    int fieldIndex = 0;
    char statusFlag = 'V';
    char timeValue[16] = "";
    char dateValue[16] = "";
    char latValue[16] = "";
    char latDir[3] = "";
    char lonValue[16] = "";
    char lonDir[3] = "";
    char speedValue[16] = "";

    while (token != nullptr) {
      if (fieldIndex == 1) {
        strncpy(timeValue, token, sizeof(timeValue) - 1);
      } else if (fieldIndex == 2) {
        statusFlag = token[0];
      } else if (fieldIndex == 3) {
        strncpy(latValue, token, sizeof(latValue) - 1);
      } else if (fieldIndex == 4) {
        strncpy(latDir, token, sizeof(latDir) - 1);
      } else if (fieldIndex == 5) {
        strncpy(lonValue, token, sizeof(lonValue) - 1);
      } else if (fieldIndex == 6) {
        strncpy(lonDir, token, sizeof(lonDir) - 1);
      } else if (fieldIndex == 7) {
        strncpy(speedValue, token, sizeof(speedValue) - 1);
      } else if (fieldIndex == 9) {
        strncpy(dateValue, token, sizeof(dateValue) - 1);
      }
      token = strtok(nullptr, ",");
      fieldIndex++;
    }

    status_.hasSentence = true;
    status_.lastUpdateMs = millis();
    status_.fixValid = (statusFlag == 'A');

    if (timeValue[0]) {
      formatUtcTime(timeValue, status_.timeUtc, sizeof(status_.timeUtc));
    }

    if (dateValue[0]) {
      formatUtcDate(dateValue, status_.dateUtc, sizeof(status_.dateUtc));
    }

    if (latValue[0] && latDir[0]) {
      status_.latitudeDeg = static_cast<float>(parseCoordinate(latValue, latDir));
    }

    if (lonValue[0] && lonDir[0]) {
      status_.longitudeDeg = static_cast<float>(parseCoordinate(lonValue, lonDir));
    }

    if (speedValue[0]) {
      const float speedKnots = atof(speedValue);
      status_.speedKmh = speedKnots * 1.852f;
    }

    return true;
  }

  EbyteGpsStatus status_;
};
