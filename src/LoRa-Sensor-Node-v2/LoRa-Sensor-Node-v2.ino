/*
 * LoRa-Sensor-Node-v2.ino
 *
 * Bench firmware for the ESP32-S3 sensor node:
 *   - Powers all rails and blinks LEDs to show GPS health.
 *   - Parses the Ebyte EWM108-GN05 NMEA stream via EbyteGpsParser.
 *   - Issues OTAA join commands to the Ebyte LoRa modem and watches for events.
 *   - Emits a human-readable status line and once-per-minute telemetry string.
 *   - Cycles peripherals and the ESP32-S3 through configurable deep-sleep windows (1 minute by default for bench tests).
 *
 * The sketch intentionally keeps logic shallow: configuration in config.h,
 * helpers for tagged logging and AT orchestration co-located, and generous
 * inline comments explaining each subsystem.
 */

#include <stdint.h>

enum class SystemFault : uint8_t;
struct FaultPattern;

#include <SPI.h>
#include <Wire.h>
#include <cstdio>
#include <cstring>
#include <esp_random.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <driver/gpio.h>

#include "Bq25186.h"
#include "config.h"
#include "EbyteGpsParser.h"
#include "LoraAtHelpers.h"
#include "SdCardLogger.h"
#include "TaggedStream.h"

namespace cfg = SensorNodeConfig;

HardwareSerial loraSerial(1);
HardwareSerial gpsSerial(2);
bool gpsSerialInitialized = false;

StreamTagState loraTag{"[LoRa] ", true};
StreamTagState gpsTag{"[GPS ] ", true};
LoraAtState loraState;

struct NullStream : public Stream {
  size_t write(uint8_t) override { return 1; }
  size_t write(const uint8_t *buffer, size_t size) override { return size; }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
};

NullStream nullStream;
Stream *debugStream = &nullStream;

inline Stream &debugSerial() {
  return *debugStream;
}

EbyteGpsParser gpsParser;
const EbyteGpsStatus &gpsStatus = gpsParser.getStatus();
SdCardLoggerState sdLoggerState;
Bq25186 charger;
constexpr Bq25186::SysMode kDefaultSysMode = Bq25186::SysMode::kAutoVinOrBattery;

constexpr SdCardLoggerConfig kSdLoggerConfig{
    cfg::kSdCsPin,
    cfg::kSdSckPin,
    cfg::kSdMisoPin,
    cfg::kSdMosiPin,
    cfg::kSdLogFilename,
};

char gpsSentenceBuffer[160];
size_t gpsSentenceLength = 0;
bool loraSendFailed = false;
bool chargerReady = false;
unsigned long lastChargerPollMs = 0;

enum class RunStage : uint8_t { AwaitGpsLock, AwaitJoin, AwaitTelemetry, AwaitPostSend };

RunStage runStage = RunStage::AwaitGpsLock;
unsigned long stageStartMs = 0;

constexpr unsigned long kGpsFreshnessWindowMs = 5000;
constexpr unsigned long kGpsCommandSettleMs = 50;

bool gpsFixIsFresh(unsigned long now) {
  return gpsStatus.fixValid &&
         gpsStatus.hasSentence &&
         (now - gpsStatus.lastUpdateMs) < kGpsFreshnessWindowMs;
}

enum class SystemFault : uint8_t { None, Gps, Lora };

struct FaultPattern {
  uint8_t flashes;
  unsigned long onMs;
  unsigned long offMs;
  unsigned long pauseMs;
};

constexpr FaultPattern kGpsFaultPattern{2, 120, 120, 500};
constexpr FaultPattern kLoraFaultPattern{3, 120, 120, 600};

struct FaultBlinkState {
  SystemFault fault = SystemFault::None;
  bool redOn = false;
  bool pausePhase = false;
  uint8_t flashesRemaining = 0;
  unsigned long lastTransitionMs = 0;
};

FaultBlinkState faultBlinkState;

constexpr char kHexDigits[] = "0123456789ABCDEF";

void setRunStage(RunStage nextStage);
bool gpsSendBinaryPacket(uint8_t groupId, uint8_t messageId, const uint8_t *payload, size_t payloadLength);
void gpsRequestSleep(uint32_t periodMs, uint8_t action);
void gpsRequestStart(uint8_t mode);
void gpsEnterBackupMode();
void gpsExitBackupMode();
void configureStatusLedPins();
inline void setStatusLed(uint8_t pin, bool high);
inline void setRedLed(bool high);
inline void setGreenLed(bool high);

void releaseHighSpeedPeripherals() {
  loraSerial.end();
  if (gpsSerialInitialized) {
    gpsSerial.flush();
  }
  gpsSerial.end();
  gpsSerialInitialized = false;
  Wire.end();
  SPI.end();
}

void configurePinsForSleep() {
  const uint8_t lowPins[] = {
      cfg::kPinI2cEnable,
      cfg::kPinLoraEnable,
      cfg::kPinGpsEnable,
      cfg::kLedRedPin,
      cfg::kLedGreenPin,
      cfg::kSdCsPin,
  };
  for (uint8_t pin : lowPins) {
    pinMode(pin, INPUT_PULLDOWN);
  }

  const gpio_num_t holdPins[] = {
      static_cast<gpio_num_t>(cfg::kPinI2cEnable),
      static_cast<gpio_num_t>(cfg::kPinLoraEnable),
      static_cast<gpio_num_t>(cfg::kPinGpsEnable),
  };
  for (gpio_num_t pin : holdPins) {
    gpio_hold_en(pin);
  }
  gpio_deep_sleep_hold_en();
}

void clearSleepHolds() {
  const gpio_num_t holdPins[] = {
      static_cast<gpio_num_t>(cfg::kPinI2cEnable),
      static_cast<gpio_num_t>(cfg::kPinLoraEnable),
      static_cast<gpio_num_t>(cfg::kPinGpsEnable),
  };
  for (gpio_num_t pin : holdPins) {
    gpio_hold_dis(pin);
  }
  gpio_deep_sleep_hold_dis();
}

void setRunStage(RunStage nextStage) {
  runStage = nextStage;
  stageStartMs = millis();
}

void configureStatusLedPins() {
  if (cfg::kEnableStatusLeds) {
    pinMode(cfg::kLedRedPin, OUTPUT);
    pinMode(cfg::kLedGreenPin, OUTPUT);
    digitalWrite(cfg::kLedRedPin, LOW);
    digitalWrite(cfg::kLedGreenPin, LOW);
  } else {
    pinMode(cfg::kLedRedPin, INPUT);
    pinMode(cfg::kLedGreenPin, INPUT);
    digitalWrite(cfg::kLedRedPin, LOW);    // Disable pull-ups.
    digitalWrite(cfg::kLedGreenPin, LOW);
  }
}

inline void setStatusLed(uint8_t pin, bool high) {
  if (!cfg::kEnableStatusLeds) {
    return;
  }
  digitalWrite(pin, high ? HIGH : LOW);
}

inline void setRedLed(bool high) {
  setStatusLed(cfg::kLedRedPin, high);
}

inline void setGreenLed(bool high) {
  setStatusLed(cfg::kLedGreenPin, high);
}

bool gpsSendBinaryPacket(uint8_t groupId,
                         uint8_t messageId,
                         const uint8_t *payload,
                         size_t payloadLength) {
  if (!gpsSerialInitialized) {
    return false;
  }

  constexpr uint8_t kStart[2] = {0xF1, 0xD9};
  uint8_t header[4] = {
      groupId,
      messageId,
      static_cast<uint8_t>(payloadLength & 0xFF),
      static_cast<uint8_t>((payloadLength >> 8) & 0xFF),
  };

  auto checksumBytes = [](const uint8_t *data, size_t length, uint8_t &ck1, uint8_t &ck2) {
    for (size_t i = 0; i < length; ++i) {
      ck1 = (ck1 + data[i]) & 0xFF;
      ck2 = (ck2 + ck1) & 0xFF;
    }
  };

  uint8_t ck1 = 0;
  uint8_t ck2 = 0;
  checksumBytes(header, sizeof(header), ck1, ck2);
  if (payload != nullptr && payloadLength > 0) {
    checksumBytes(payload, payloadLength, ck1, ck2);
  }

  gpsSerial.write(kStart, sizeof(kStart));
  gpsSerial.write(header, sizeof(header));
  if (payload != nullptr && payloadLength > 0) {
    gpsSerial.write(payload, payloadLength);
  }
  const uint8_t checksum[2] = {ck1, ck2};
  gpsSerial.write(checksum, sizeof(checksum));
  gpsSerial.flush();
  return true;
}

void gpsRequestSleep(uint32_t periodMs, uint8_t action) {
  if (!gpsSerialInitialized || periodMs == 0) {
    return;
  }
  uint8_t payload[5];
  payload[0] = static_cast<uint8_t>(periodMs & 0xFF);
  payload[1] = static_cast<uint8_t>((periodMs >> 8) & 0xFF);
  payload[2] = static_cast<uint8_t>((periodMs >> 16) & 0xFF);
  payload[3] = static_cast<uint8_t>((periodMs >> 24) & 0xFF);
  payload[4] = action;

  debugSerial().print("[GPS ] CFG-SLEEP action=0x");
  debugSerial().print(action, HEX);
  debugSerial().print(" for ");
  debugSerial().print(periodMs);
  debugSerial().println(" ms.");

  if (gpsSendBinaryPacket(0x06, 0x41, payload, sizeof(payload))) {
    delay(kGpsCommandSettleMs);
  }
}

void gpsRequestStart(uint8_t mode) {
  if (!gpsSerialInitialized || mode == 0xFF) {
    return;
  }
  uint8_t payload = mode;
  debugSerial().print("[GPS ] CFG-CMD mode=0x");
  debugSerial().print(mode, HEX);
  debugSerial().println(" (wake).");
  if (gpsSendBinaryPacket(0x06, 0x40, &payload, sizeof(payload))) {
    delay(kGpsCommandSettleMs);
  }
}

void gpsEnterBackupMode() {
  if (!cfg::kGpsPowerConfig.enabled) {
    return;
  }
  gpsRequestSleep(cfg::kGpsPowerConfig.sleepDurationMs,
                  cfg::kGpsPowerConfig.sleepAction);
}

void gpsExitBackupMode() {
  if (!cfg::kGpsPowerConfig.enabled) {
    return;
  }
  gpsRequestStart(cfg::kGpsPowerConfig.wakeMode);
}

void setPeripheralPower(bool enabled) {
  const int level = enabled ? HIGH : LOW;
  digitalWrite(cfg::kPinI2cEnable, level);
  digitalWrite(cfg::kPinLoraEnable, level);
  digitalWrite(cfg::kPinGpsEnable, level);
}

void powerUpPeripherals() {
  debugSerial().println("[Pwr ] Enabling power rails.");
  setPeripheralPower(true);
  delay(cfg::kPeripheralWarmupMs);
}

void powerDownPeripherals() {
  debugSerial().println("[Pwr ] Disabling power rails.");
  setPeripheralPower(false);
  setRedLed(false);
  setGreenLed(false);
}

void logWakeReason() {
  const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  const char *description = "power-on/reset";
  switch (cause) {
    case ESP_SLEEP_WAKEUP_TIMER:
      description = "RTC timer";
      break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
      description = "undefined";
      break;
    case ESP_SLEEP_WAKEUP_EXT0:
    case ESP_SLEEP_WAKEUP_EXT1:
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
    case ESP_SLEEP_WAKEUP_ULP:
    case ESP_SLEEP_WAKEUP_GPIO:
      description = "external trigger";
      break;
    default:
      break;
  }
  debugSerial().print("[Pwr ] Wake cause: ");
  debugSerial().println(description);
}

void enterDeepSleep(const char *reason) {
  if (!cfg::kEnableDeepSleep) {
    debugSerial().print("[Pwr ] Deep sleep disabled; would have slept because: ");
    debugSerial().println(reason ? reason : "(unspecified)");
    setRunStage(RunStage::AwaitGpsLock);
    return;
  }
  debugSerial().print("[Pwr ] Entering deep sleep for ");
  debugSerial().print(cfg::kDeepSleepIntervalUs / 1000000ULL);
  debugSerial().print(" seconds (reason: ");
  debugSerial().print(reason);
  debugSerial().println(")");
  gpsEnterBackupMode();
  if (cfg::kEnableDebugSerial) {
    Serial.flush();
  }
  releaseHighSpeedPeripherals();
  if (cfg::kEnableDebugSerial) {
    Serial.end();
  }
  powerDownPeripherals();
  configurePinsForSleep();
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
#ifdef ESP_PD_DOMAIN_RTC_FAST_MEM
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
#endif
#ifdef ESP_PD_DOMAIN_RTC_SLOW_MEM
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
#endif
  esp_sleep_enable_timer_wakeup(cfg::kDeepSleepIntervalUs);
  esp_deep_sleep_start();
}

void handleGpsLockStage() {
  const unsigned long now = millis();
  if (gpsFixIsFresh(now)) {
    debugSerial().println("[GPS ] Fix acquired; exiting cold start.");
    setRunStage(RunStage::AwaitJoin);
    return;
  }

  const bool coldStartElapsed =
      cfg::kGpsColdStartMs == 0 || (now - stageStartMs) >= cfg::kGpsColdStartMs;
  if (!coldStartElapsed) {
    return;
  }

  if (cfg::kGpsLockTimeoutMs > 0 &&
      (now - stageStartMs) >= cfg::kGpsLockTimeoutMs) {
    debugSerial().println("[GPS ] Cold start timeout; continuing without valid fix.");
    setRunStage(RunStage::AwaitJoin);
  }
}

void handleJoinStage() {
  if (!loraJoined(loraState)) {
    loraStartJoin(loraState, cfg::kLoraJoinRetryMs);
  }

  if (loraJoined(loraState)) {
    debugSerial().println("[LoRa] Join complete; sending telemetry.");
    setRunStage(RunStage::AwaitTelemetry);
    return;
  }

  const unsigned long now = millis();
  if (cfg::kJoinTimeoutMs > 0 &&
      (now - stageStartMs) >= cfg::kJoinTimeoutMs) {
    debugSerial().println("[LoRa] Join timeout; sleeping before retry.");
    enterDeepSleep("join timeout");
  }
}

void handleTelemetryStage() {
  const unsigned long now = millis();
  if (!gpsFixIsFresh(now)) {
    debugSerial().println("[GPS ] No fresh fix; sending best-effort telemetry.");
  }

  if (sendTelemetryFrame()) {
    setRunStage(RunStage::AwaitPostSend);
  } else {
    debugSerial().println("[LoRa] Telemetry failed; entering sleep to retry next wake.");
    enterDeepSleep("telemetry failure");
  }
}

void handlePostSendStage() {
  const unsigned long now = millis();
  if (now - stageStartMs >= cfg::kPostSendHoldMs) {
    enterDeepSleep("telemetry complete");
  }
}

// State flow cheat sheet:
//   AwaitGpsLock  -> Wait for fix (respect cold-start window).
//   AwaitJoin     -> Retry OTAA join until accepted (or timeout -> sleep).
//   AwaitTelemetry-> Send telemetry immediately after join.
//   AwaitPostSend -> Hold a few ms for logs, then deep sleep.
void runStateMachine() {
  switch (runStage) {
    case RunStage::AwaitGpsLock:
      handleGpsLockStage();
      break;
    case RunStage::AwaitJoin:
      handleJoinStage();
      break;
    case RunStage::AwaitTelemetry:
      handleTelemetryStage();
      break;
    case RunStage::AwaitPostSend:
      handlePostSendStage();
      break;
  }
}

void printChargerTelemetry(const Bq25186::Telemetry &telemetry) {
  debugSerial().print("[Chg ] SYS=");
  debugSerial().print(Bq25186::sysModeDescription(telemetry.sysMode));
  debugSerial().print(" | VbatTarget=");
  debugSerial().print(telemetry.batteryRegVoltage, 3);
  debugSerial().print(" V | IchgTarget=");
  debugSerial().print(telemetry.fastChargeCurrentMa, 0);
  debugSerial().print(" mA | ILIM=");
  debugSerial().print(telemetry.inputCurrentLimitMa, 0);
  debugSerial().print(" mA | Charge=");
  debugSerial().print(Bq25186::chargeStateDescription(telemetry.stat0.chargeState));
  debugSerial().print(" (");
  debugSerial().print(telemetry.chargeEnabled ? "enabled" : "disabled");
  debugSerial().print(") | TS=");
  debugSerial().print(Bq25186::tsStatusDescription(telemetry.stat1.tsStatus));
  debugSerial().print(" | PG=");
  debugSerial().print(telemetry.stat0.vinPowerGood ? "Y" : "N");
  debugSerial().print(" | WDT=");
  debugSerial().print(telemetry.watchdogEnabled ? "Y" : "N");
  if (telemetry.faults.any()) {
    debugSerial().print(" | Faults:");
    if (telemetry.faults.tsFault) {
      debugSerial().print(" TS");
    }
    if (telemetry.faults.ilimFault) {
      debugSerial().print(" ILIM");
    }
    if (telemetry.faults.vdppmFault) {
      debugSerial().print(" VDPPM");
    }
    if (telemetry.faults.vindpmFault) {
      debugSerial().print(" VINDPM");
    }
    if (telemetry.faults.thermalRegulationFlag) {
      debugSerial().print(" THERM");
    }
    if (telemetry.faults.vinOvpFault) {
      debugSerial().print(" VIN_OVP");
    }
    if (telemetry.faults.buvloFault) {
      debugSerial().print(" BUVLO");
    }
    if (telemetry.faults.batOcpFault) {
      debugSerial().print(" BAT_OCP");
    }
  }
  debugSerial().println();
}

void pollCharger() {
  if (!chargerReady) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastChargerPollMs < 2000) {
    return;
  }
  lastChargerPollMs = now;

  Bq25186::Telemetry telemetry;
  if (!charger.readTelemetry(telemetry)) {
    debugSerial().println("[Chg ] Failed to read BQ25186 registers.");
    return;
  }
  printChargerTelemetry(telemetry);
}

// Converts an ASCII payload into uppercase hex so it can be fed to AT+SEND.
void asciiToHex(const char *input, char *output, size_t outputSize) {
  size_t outIndex = 0;
  for (size_t i = 0; input[i] != '\0'; ++i) {
    const uint8_t value = static_cast<uint8_t>(input[i]);
    if (outIndex + 2 >= outputSize) {
      break;
    }
    output[outIndex++] = kHexDigits[(value >> 4) & 0x0F];
    output[outIndex++] = kHexDigits[value & 0x0F];
  }
  output[outIndex] = '\0';
}

// Prints a single-line summary of the most recent GPS sentence.
void printGpsSummary() {
  char summary[196];
  snprintf(summary,
           sizeof(summary),
           "[GPS ] UTC %s %s | %s | Sats %u | Lat %.5f | Lon %.5f | Alt %.1f m | Speed %.2f km/h",
           gpsStatus.timeUtc,
           gpsStatus.dateUtc,
           gpsStatus.fixValid ? "Fix OK" : "No Fix",
           gpsStatus.satelliteCount,
           gpsStatus.latitudeDeg,
           gpsStatus.longitudeDeg,
           gpsStatus.altitudeMeters,
           gpsStatus.speedKmh);

  debugSerial().println(summary);
  sdLoggerLogLine(sdLoggerState, summary);
  sdLoggerLogCsv(sdLoggerState, gpsStatus);
}

// Dumps the provisioned LoRaWAN identifiers so they are captured in the log.
void printLoraInfo() {
  debugSerial().println("[LoRa] Provisioned credentials:");
  debugSerial().print("[LoRa]   DevEUI : ");
  debugSerial().println(cfg::kLoraCredentials.devEui);
  debugSerial().print("[LoRa]   AppEUI : ");
  debugSerial().println(cfg::kLoraCredentials.appEui);
  debugSerial().print("[LoRa]   AppKey : ");
  debugSerial().println(cfg::kLoraCredentials.appKey);
  debugSerial().print("[LoRa] Will transmit with AT+SEND=");
  debugSerial().print(cfg::kLoraRadioSettings.sendPort);
  debugSerial().println(":<HEX_PAYLOAD>");
}

// Formats the telemetry payload and hands it to the LoRa helper.
bool sendTelemetryFrame() {
  if (!loraJoined(loraState)) {
    debugSerial().println("[LoRa] Skipping telemetry: waiting for +EVT:JOINED.");
    loraStartJoin(loraState, cfg::kLoraJoinRetryMs);
    return false;
  }

  const float temperatureC = random(180, 350) / 10.0f;   // Simulated 18-35 C
  const float humidityPct = random(300, 900) / 10.0f;    // Simulated 30-90 %
  const char *fixState = gpsStatus.fixValid ? "FIX" : "NO_FIX";

  char payload[160];
  snprintf(payload, sizeof(payload),
           "ALT=%.1f,TEMP=%.1f,HUM=%.1f,STATUS=%s",
           gpsStatus.altitudeMeters, temperatureC, humidityPct, fixState);

  debugSerial().print("[LoRa] Sending telemetry: ");
  debugSerial().println(payload);

  char hexPayload[320];
  asciiToHex(payload, hexPayload, sizeof(hexPayload));

  if (!loraSendHexPayload(loraState,
                          cfg::kLoraRadioSettings.sendPort,
                          hexPayload,
                          cfg::kLoraRadioSettings.sendResponse,
                          6000)) {
    debugSerial().println("[LoRa] Telemetry send failed.");
    loraSendFailed = true;
    return false;
  }
  loraSendFailed = false;
  return true;
}

// Fires once per minute as long as we've seen at least one GPS fix.
void processGpsSentence(const char *sentence) {
  if (gpsParser.processSentence(sentence)) {
    printGpsSummary();
  }
}

// Collects each GPS byte until CR/LF, optionally echoing the raw stream.
void handleGpsByte(int byteValue) {
  if (cfg::kShowRawGps) {
    printTaggedByte(debugSerial(), gpsTag, byteValue);
  }

  if (byteValue == '\n') {
    gpsSentenceBuffer[gpsSentenceLength] = '\0';
    if (gpsSentenceLength > 0) {
      processGpsSentence(gpsSentenceBuffer);
    }
    gpsSentenceLength = 0;
  } else if (byteValue != '\r') {
    if (gpsSentenceLength < sizeof(gpsSentenceBuffer) - 1) {
      gpsSentenceBuffer[gpsSentenceLength++] = static_cast<char>(byteValue);
    } else {
      gpsSentenceLength = 0;
    }
  }
}

const FaultPattern &patternFor(SystemFault fault) {
  return (fault == SystemFault::Lora) ? kLoraFaultPattern : kGpsFaultPattern;
}

SystemFault determineFault(unsigned long now) {
  const bool gpsHealthy = gpsFixIsFresh(now);
  const bool loraIsJoined = loraJoined(loraState);
  const bool withinJoinGrace = !loraIsJoined &&
                               (loraState.lastJoinAttemptMs == 0 ||
                                (now - loraState.lastJoinAttemptMs) < cfg::kLoraJoinRetryMs);
  const bool loraHealthy = !loraSendFailed && (loraIsJoined || withinJoinGrace);

  if (!loraHealthy) {
    return SystemFault::Lora;
  }
  if (!gpsHealthy) {
    return SystemFault::Gps;
  }
  return SystemFault::None;
}

void resetFaultBlinkState(SystemFault fault, unsigned long now) {
  faultBlinkState.fault = fault;
  faultBlinkState.redOn = false;
  faultBlinkState.pausePhase = false;
  faultBlinkState.flashesRemaining =
      (fault == SystemFault::None) ? 0 : patternFor(fault).flashes;
  faultBlinkState.lastTransitionMs = now;
  setRedLed(false);
}

void driveFaultPattern(SystemFault fault, unsigned long now) {
  if (!cfg::kEnableStatusLeds) {
    return;
  }

  if (fault == SystemFault::None) {
    setRedLed(false);
    setGreenLed(true);
    resetFaultBlinkState(SystemFault::None, now);
    return;
  }

  setGreenLed(false);

  if (fault != faultBlinkState.fault) {
    resetFaultBlinkState(fault, now);
  }

  const FaultPattern &pattern = patternFor(fault);

  if (faultBlinkState.pausePhase) {
    if (now - faultBlinkState.lastTransitionMs >= pattern.pauseMs) {
      faultBlinkState.pausePhase = false;
      faultBlinkState.flashesRemaining = pattern.flashes;
      faultBlinkState.lastTransitionMs = now;
    }
    return;
  }

  if (faultBlinkState.redOn) {
    if (now - faultBlinkState.lastTransitionMs >= pattern.onMs) {
      faultBlinkState.redOn = false;
      faultBlinkState.lastTransitionMs = now;
      setRedLed(false);
      if (faultBlinkState.flashesRemaining > 0) {
        faultBlinkState.flashesRemaining--;
      }
      if (faultBlinkState.flashesRemaining == 0) {
        faultBlinkState.pausePhase = true;
        faultBlinkState.lastTransitionMs = now;
      }
    }
    return;
  }

  if (faultBlinkState.flashesRemaining == 0) {
    faultBlinkState.pausePhase = true;
    faultBlinkState.lastTransitionMs = now;
    return;
  }

  if (now - faultBlinkState.lastTransitionMs >= pattern.offMs) {
    faultBlinkState.redOn = true;
    faultBlinkState.lastTransitionMs = now;
    setRedLed(true);
  }
}

void updateStatusIndicators() {
  if (!cfg::kEnableStatusLeds) {
    return;
  }
  const unsigned long now = millis();
  const SystemFault fault = determineFault(now);
  driveFaultPattern(fault, now);
}

// Classic Arduino-style loop replacement:
//   1. Pump LoRa/GPS UARTs.
//   2. Drive the run-state machine (GPS lock -> join -> telemetry -> post-send).
//   3. Update LED fault patterns and charger telemetry.
//   4. Idle briefly so the bench console is readable.
void runCycle() {
  while (true) {
    loraPoll(loraState);

    while (gpsSerial.available()) {
      const int byteValue = gpsSerial.read();
      handleGpsByte(byteValue);
    }

    runStateMachine();
    updateStatusIndicators();
    pollCharger();
    delay(5);
  }
}

// setup():
//   - Configures rail enables, indicator pins, and the debug console.
//   - Powers peripherals, starts UART/I2C, and wakes the Allystar module.
//   - Initializes helpers (LoRa AT state, SD logger, charger).
//   - Prints configuration context and launches the state machine by
//     entering RunStage::AwaitGpsLock.
void setup() {
  pinMode(cfg::kPinI2cEnable, OUTPUT);
  pinMode(cfg::kPinLoraEnable, OUTPUT);
  pinMode(cfg::kPinGpsEnable, OUTPUT);

  configureStatusLedPins();

  clearSleepHolds();

  digitalWrite(cfg::kPinI2cEnable, LOW);
  digitalWrite(cfg::kPinLoraEnable, LOW);
  digitalWrite(cfg::kPinGpsEnable, LOW);

  if (cfg::kEnableDebugSerial) {
    Serial.begin(115200);
    debugStream = &Serial;
  } else {
    debugStream = &nullStream;
  }
  logWakeReason();

  powerUpPeripherals();

  loraSerial.begin(cfg::kLoraBaud, SERIAL_8N1, cfg::kLoraRxPin, cfg::kLoraTxPin);
  gpsSerial.begin(cfg::kGpsBaud, SERIAL_8N1, cfg::kGpsRxPin, cfg::kGpsTxPin);
  gpsSerialInitialized = true;
  Wire.begin(cfg::kI2cSdaPin, cfg::kI2cSclPin);
  Wire.setClock(400000);
  randomSeed(esp_random());

  gpsExitBackupMode();

  loraInitState(loraState, loraSerial, debugSerial(), loraTag);
  sdLoggerBegin(sdLoggerState, kSdLoggerConfig, debugSerial());
  chargerReady = charger.begin(Wire);
  if (chargerReady) {
    debugSerial().print("[Chg ] BQ25186 detected at 0x");
    debugSerial().println(Bq25186::kDefaultI2cAddress, HEX);
    if (charger.setSysMode(kDefaultSysMode)) {
      debugSerial().print("[Chg ] SYS mode set to ");
      debugSerial().println(Bq25186::sysModeDescription(kDefaultSysMode));
    } else {
      debugSerial().println("[Chg ] Failed to program SYS mode.");
    }
    if (cfg::kChargeTargetCurrentMa > 0 &&
        charger.setFastChargeCurrentMa(cfg::kChargeTargetCurrentMa)) {
      debugSerial().print("[Chg ] Fast charge current set to ");
      debugSerial().print(cfg::kChargeTargetCurrentMa, 0);
      debugSerial().println(" mA.");
    } else if (cfg::kChargeTargetCurrentMa > 0) {
      debugSerial().println("[Chg ] Failed to set fast charge current.");
    }
    if (cfg::kInputCurrentLimitMa > 0 &&
        charger.setInputCurrentLimitMa(cfg::kInputCurrentLimitMa)) {
      debugSerial().print("[Chg ] Input current limit set to ");
      debugSerial().print(cfg::kInputCurrentLimitMa, 0);
      debugSerial().println(" mA.");
    } else if (cfg::kInputCurrentLimitMa > 0) {
      debugSerial().println("[Chg ] Failed to set input current limit.");
    }
  } else {
    debugSerial().println("[Chg ] Unable to communicate with BQ25186.");
  }

  debugSerial().println();
  debugSerial().println("Sensor Node Test - GPS + LoRa");
  debugSerial().println("Monitoring GPS fix status and forwarding LoRa AT commands.");
  debugSerial().print("Auto-joining LoRaWAN, sending telemetry, then ");
  if (cfg::kEnableDeepSleep) {
    debugSerial().print("deep-sleeping for ");
    debugSerial().print(cfg::kDeepSleepIntervalUs / 1000000ULL);
    debugSerial().println(" seconds.");
  } else {
    debugSerial().println("looping without ESP32 deep sleep (kEnableDeepSleep=false).");
  }
  debugSerial().println();
  printLoraInfo();

  debugSerial().print("[GPS ] Allowing ");
  debugSerial().print(cfg::kGpsColdStartMs / 1000);
  debugSerial().println(" s of cold start before LoRa join.");
  if (cfg::kGpsLockTimeoutMs > 0) {
    debugSerial().print("[GPS ] Will proceed without a fix after ");
    debugSerial().print(cfg::kGpsLockTimeoutMs / 1000);
    debugSerial().println(" s if necessary.");
  }
  if (cfg::kGpsPowerConfig.enabled &&
      cfg::kGpsPowerConfig.sleepDurationMs > 0) {
    debugSerial().print("[GPS ] Will issue CFG-SLEEP action=0x");
    debugSerial().print(cfg::kGpsPowerConfig.sleepAction, HEX);
    debugSerial().print(" for ");
    debugSerial().print(cfg::kGpsPowerConfig.sleepDurationMs);
    debugSerial().println(" ms before rail-off.");
  }
  if (cfg::kGpsPowerConfig.enabled &&
      cfg::kGpsPowerConfig.wakeMode != 0xFF) {
    debugSerial().print("[GPS ] Wake command set to CFG-CMD mode=0x");
    debugSerial().print(cfg::kGpsPowerConfig.wakeMode, HEX);
    debugSerial().println(".");
  }
  if (!cfg::kGpsPowerConfig.enabled) {
    debugSerial().println("[GPS ] Allystar sleep/wake commands disabled (kGpsPowerConfig.enabled=false).");
  }

  setRunStage(RunStage::AwaitGpsLock);

  runCycle();
}

void loop() {}
