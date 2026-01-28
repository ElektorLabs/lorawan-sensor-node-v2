#pragma once

#include <Arduino.h>

namespace SensorNodeConfig {

// Peripheral power-control GPIOs.
constexpr uint8_t kPinI2cEnable = 38;
constexpr uint8_t kPinLoraEnable = 6;
constexpr uint8_t kPinGpsEnable = 7;

// Indicator LEDs.
constexpr uint8_t kLedRedPin = 48;
constexpr uint8_t kLedGreenPin = 47;

// I2C bus shared with the charger IC.
constexpr int kI2cSdaPin = 10;
constexpr int kI2cSclPin = 9;

// SD card over SPI.
constexpr uint8_t kSdMosiPin = 11;  // MOSI (SD-MO)
constexpr uint8_t kSdSckPin = 12;   // SCK
constexpr uint8_t kSdMisoPin = 13;  // MISO (SD-MI)
constexpr uint8_t kSdCsPin = 8;     // Chip select
constexpr const char *kSdLogFilename = "/gps-log.csv";

// UART assignments.
constexpr int kLoraRxPin = 4;
constexpr int kLoraTxPin = 5;
constexpr uint32_t kLoraBaud = 115200;

constexpr int kGpsRxPin = 21;
constexpr int kGpsTxPin = 14;
constexpr uint32_t kGpsBaud = 115200;

// Telemetry + power cadence.
constexpr unsigned long kJoinTimeoutMs = 0;              // 0 = wait indefinitely for OTAA.
constexpr unsigned long kSensorAcquisitionMs = 15000;    // Fallback if GPS never reports.
constexpr unsigned long kPostSendHoldMs = 3000;          // Log/flush delay before sleeping.
constexpr unsigned long kPeripheralWarmupMs = 1500;      // Allow power-rail settling.
constexpr uint64_t kDeepSleepIntervalUs = 600ULL * 1000000ULL;  // 1 minute test interval.
constexpr unsigned long kLoraJoinRetryMs = 15000;        // Retry OTAA command every 15 s.
constexpr unsigned long kGpsColdStartMs = 60000;         // Minimum wait before attempting LoRa join.
constexpr unsigned long kGpsLockTimeoutMs = 180000;      // Proceed even without fix after 3 minutes.

// Raw GPS logging toggle.
constexpr bool kShowRawGps = false;
constexpr bool kEnableDebugSerial = true;
constexpr bool kEnableDeepSleep = true;  // Set false for bench loops without ESP32 deep sleep.
constexpr bool kEnableStatusLeds = false; // Disable to force both LEDs off for low-power tests.
constexpr float kChargeTargetCurrentMa = 520.0f;   // Mid-scale fast-charge current (5-1000 mA range).
constexpr float kInputCurrentLimitMa = 665.0f;     // BQ25186 ILIM step just above 500 mA.

// Allystar GNSS power profile (CFG-SLEEP / CFG-CMD per T-5-2302 spec).
struct AllystarGpsPowerConfig {
  bool enabled;              // Master toggle for Allystar sleep/wake commands.
  uint32_t sleepDurationMs;  // Used when sending CFG-SLEEP (0 disables the command).
  uint8_t sleepAction;       // 0: sleep, 1: deep sleep, 3: power down, etc.
  uint8_t wakeMode;          // CFG-CMD mode (0x11 = Start, 0x03 = hot start, etc.).
};

constexpr AllystarGpsPowerConfig kGpsPowerConfig{
    true,    // Enable GPS sleep/wake commands by default.
    90000,   // Keep Allystar in deep sleep for 90 seconds while rails are off.
    0x01,    // Use action=1 (deep sleep) before cutting power.
    0x11,    // Issue CFG-CMD start to wake after rails return.
};

// LoRaWAN identifiers/keys.
struct LoraCredentials {
  const char *devEui;
  const char *appEui;
  const char *appKey;
};

constexpr LoraCredentials kLoraCredentials{
    "xxxxx",
    "xxxxx",
    "xxxxx",
};

// LoRaWAN radio settings.
struct LoraRadioSettings {
  const char *region;
  const char *channelPlan;
  int defaultPort;
  int sendPort;
  const char *sendResponse;
};

constexpr LoraRadioSettings kLoraRadioSettings{
    "EU868",
    "0-2",
    2,
    2,
    "OK",
};

}  // namespace SensorNodeConfig
