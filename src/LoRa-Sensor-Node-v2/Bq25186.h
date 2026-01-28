/*
 * Bq25186.h
 *
 * Arduino-friendly helper for the TI BQ25186 1-cell charger.
 * Uses the register definitions from docs/bq25186(.pdf/.md).
 * Provides register access, decoded telemetry, and helpers
 * for entering ship/shutdown modes or tuning charge targets.
 *
 * Example:
 *   #include "Bq25186.h"
 *   Bq25186 charger;
 *   void setup() {
 *     Wire.begin();
 *     Serial.begin(115200);
 *     if (charger.begin(Wire)) {
 *       Bq25186::Telemetry t;
 *       if (charger.readTelemetry(t)) {
 *         Serial.println(t.batteryRegVoltage);
 *       }
 *     }
 *   }
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class Bq25186 {
 public:
  static constexpr uint8_t kDefaultI2cAddress = 0x6A;

  enum Register : uint8_t {
    kRegStat0 = 0x00,
    kRegStat1 = 0x01,
    kRegFlag0 = 0x02,
    kRegVbatCtrl = 0x03,
    kRegIchgCtrl = 0x04,
    kRegChargeCtrl0 = 0x05,
    kRegChargeCtrl1 = 0x06,
    kRegIcCtrl = 0x07,
    kRegTmrIlim = 0x08,
    kRegShipReset = 0x09,
    kRegSysReg = 0x0A,
    kRegTsControl = 0x0B,
    kRegMaskId = 0x0C,
  };

  enum class ChargeState : uint8_t {
    kNotCharging = 0,
    kConstantCurrent = 1,
    kConstantVoltage = 2,
    kChargeCompleteOrDisabled = 3,
  };

  enum class TsStatus : uint8_t {
    kNormal = 0,
    kColdOrHot = 1,
    kCool = 2,
    kWarm = 3,
  };

  enum class SysMode : uint8_t {
    kAutoVinOrBattery = 0,  // SYS from VIN if present, else VBAT.
    kBatteryOnly = 1,       // Force SYS from VBAT even if VIN valid.
    kSysOffHighZ = 2,       // SYS disconnected, left floating.
    kSysOffPulldown = 3,    // SYS disconnected with pulldown.
  };

  enum class ShipAction : uint8_t {
    kNone = 0,
    kShutdown = 1,
    kShip = 2,
    kHardwareReset = 3,
  };

  struct Stat0Flags {
    bool tsOpen = false;
    ChargeState chargeState = ChargeState::kNotCharging;
    bool ilimActive = false;
    bool vdppmActive = false;
    bool vindpmActive = false;
    bool thermalRegulation = false;
    bool vinPowerGood = false;
  };

  struct Stat1Flags {
    bool vinOvp = false;
    bool batteryUvlo = false;
    TsStatus tsStatus = TsStatus::kNormal;
    bool safetyTimerFault = false;
    bool wake1 = false;
    bool wake2 = false;
  };

  struct FaultFlags {
    bool tsFault = false;
    bool ilimFault = false;
    bool vdppmFault = false;
    bool vindpmFault = false;
    bool thermalRegulationFlag = false;
    bool vinOvpFault = false;
    bool buvloFault = false;
    bool batOcpFault = false;

    bool any() const {
      return tsFault || ilimFault || vdppmFault || vindpmFault ||
             thermalRegulationFlag || vinOvpFault || buvloFault || batOcpFault;
    }
  };

  struct RegisterBlock {
    uint8_t stat0 = 0;
    uint8_t stat1 = 0;
    uint8_t flag0 = 0;
    uint8_t vbatCtrl = 0;
    uint8_t ichgCtrl = 0;
    uint8_t chargeCtrl0 = 0;
    uint8_t chargeCtrl1 = 0;
    uint8_t icCtrl = 0;
    uint8_t tmrIlim = 0;
    uint8_t shipReset = 0;
    uint8_t sysReg = 0;
    uint8_t tsControl = 0;
    uint8_t maskId = 0;
  };

  struct Telemetry {
    RegisterBlock raw;
    Stat0Flags stat0;
    Stat1Flags stat1;
    FaultFlags faults;
    SysMode sysMode = SysMode::kAutoVinOrBattery;
    float batteryRegVoltage = 0.0f;
    float fastChargeCurrentMa = 0.0f;
    float inputCurrentLimitMa = 0.0f;
    bool chargeEnabled = true;
    bool watchdogEnabled = false;
  };

  bool begin(TwoWire &wirePort, uint8_t address = kDefaultI2cAddress) {
    wire_ = &wirePort;
    address_ = address;
    uint8_t deviceId = 0;
    return readRegister(kRegMaskId, deviceId);
  }

  bool readStat0(Stat0Flags &flags) const {
    uint8_t raw = 0;
    if (!readRegister(kRegStat0, raw)) {
      return false;
    }
    flags = decodeStat0(raw);
    return true;
  }

  bool readStat1(Stat1Flags &flags) const {
    uint8_t raw = 0;
    if (!readRegister(kRegStat1, raw)) {
      return false;
    }
    flags = decodeStat1(raw);
    return true;
  }

  bool readFaults(FaultFlags &faults) const {
    uint8_t raw = 0;
    if (!readRegister(kRegFlag0, raw)) {
      return false;
    }
    faults = decodeFaults(raw);
    return true;
  }

  bool readRegisterBlock(RegisterBlock &block) const {
    uint8_t buffer[13] = {};
    if (!readRegisters(kRegStat0, buffer, sizeof(buffer))) {
      return false;
    }
    block.stat0 = buffer[0];
    block.stat1 = buffer[1];
    block.flag0 = buffer[2];
    block.vbatCtrl = buffer[3];
    block.ichgCtrl = buffer[4];
    block.chargeCtrl0 = buffer[5];
    block.chargeCtrl1 = buffer[6];
    block.icCtrl = buffer[7];
    block.tmrIlim = buffer[8];
    block.shipReset = buffer[9];
    block.sysReg = buffer[10];
    block.tsControl = buffer[11];
    block.maskId = buffer[12];
    return true;
  }

  bool readTelemetry(Telemetry &telemetry) const {
    if (!readRegisterBlock(telemetry.raw)) {
      return false;
    }
    telemetry.stat0 = decodeStat0(telemetry.raw.stat0);
    telemetry.stat1 = decodeStat1(telemetry.raw.stat1);
    telemetry.faults = decodeFaults(telemetry.raw.flag0);
    telemetry.sysMode = decodeSysMode(telemetry.raw.sysReg);
    telemetry.batteryRegVoltage = decodeBatteryRegVoltage(telemetry.raw.vbatCtrl);
    telemetry.fastChargeCurrentMa = decodeFastChargeCurrent(telemetry.raw.ichgCtrl);
    telemetry.inputCurrentLimitMa = decodeInputCurrentLimit(telemetry.raw.tmrIlim);
    telemetry.chargeEnabled = (telemetry.raw.ichgCtrl & kIchgDisableMask) == 0;
    const uint8_t watchdogSel = telemetry.raw.icCtrl & kIcCtrlWatchdogMask;
    const bool icWatchdogEnabled = watchdogSel != kIcCtrlWatchdogMask;  // 0b11 disables.
    const bool sysWatchdogEnabled =
        (telemetry.raw.sysReg & kSysRegWatchdog15sMask) != 0;
    telemetry.watchdogEnabled = icWatchdogEnabled || sysWatchdogEnabled;
    return true;
  }

  bool writeRegister(uint8_t reg, uint8_t value) const {
    if (!wire_) {
      return false;
    }
    wire_->beginTransmission(address_);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
  }

  bool readRegister(uint8_t reg, uint8_t &value) const {
    if (!wire_) {
      return false;
    }
    wire_->beginTransmission(address_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) {
      return false;
    }
    if (wire_->requestFrom(address_, static_cast<uint8_t>(1)) != 1) {
      return false;
    }
    value = wire_->read();
    return true;
  }

  bool readRegisters(uint8_t startReg, uint8_t *buffer, size_t length) const {
    if (!wire_ || buffer == nullptr || length == 0) {
      return false;
    }
    wire_->beginTransmission(address_);
    wire_->write(startReg);
    if (wire_->endTransmission(false) != 0) {
      return false;
    }
    const uint8_t bytesRequested = static_cast<uint8_t>(length);
    if (wire_->requestFrom(address_, bytesRequested) != bytesRequested) {
      return false;
    }
    for (size_t i = 0; i < length; ++i) {
      buffer[i] = wire_->read();
    }
    return true;
  }

  bool modifyRegister(uint8_t reg, uint8_t clearMask, uint8_t setMask) const {
    uint8_t value = 0;
    if (!readRegister(reg, value)) {
      return false;
    }
    value &= ~clearMask;
    value |= setMask;
    return writeRegister(reg, value);
  }

  bool setBatteryRegulationVoltage(float volts) const {
    const uint8_t code = encodeBatteryRegVoltage(volts) & 0x7F;
    return modifyRegister(kRegVbatCtrl, 0x7F, code);
  }

  bool setFastChargeCurrentMa(float milliAmps) const {
    const uint8_t code = encodeFastChargeCurrent(milliAmps) & 0x7F;
    return modifyRegister(kRegIchgCtrl, 0x7F, code);
  }

  bool enableCharging(bool enable) const {
    return modifyRegister(kRegIchgCtrl, kIchgDisableMask, enable ? 0 : kIchgDisableMask);
  }

  bool setInputCurrentLimitMa(float milliAmps) const {
    const uint8_t code = encodeInputCurrentLimit(milliAmps);
    return modifyRegister(kRegTmrIlim, kTmrIlimIlimMask, code);
  }

  bool setSysMode(SysMode mode) const {
    return modifyRegister(kRegSysReg,
                          static_cast<uint8_t>(kSysModeMask),
                          static_cast<uint8_t>(static_cast<uint8_t>(mode) << kSysModeShift));
  }

  bool requestShipAction(ShipAction action) const {
    const uint8_t setBits =
        (static_cast<uint8_t>(action) << kShipActionShift) & kShipActionMask;
    return modifyRegister(kRegShipReset, kShipActionMask, setBits);
  }

  bool enterShipMode() const { return requestShipAction(ShipAction::kShip); }
  bool enterShutdownMode() const { return requestShipAction(ShipAction::kShutdown); }
  bool hardwareReset() const { return requestShipAction(ShipAction::kHardwareReset); }

  bool softwareReset() const {
    return modifyRegister(kRegShipReset, kShipRegisterResetMask, kShipRegisterResetMask);
  }

  static const char *chargeStateDescription(ChargeState state) {
    switch (state) {
      case ChargeState::kNotCharging:
        return "Idle";
      case ChargeState::kConstantCurrent:
        return "CC";
      case ChargeState::kConstantVoltage:
        return "CV";
      case ChargeState::kChargeCompleteOrDisabled:
        return "Done";
    }
    return "Unknown";
  }

  static const char *tsStatusDescription(TsStatus status) {
    switch (status) {
      case TsStatus::kNormal:
        return "Normal";
      case TsStatus::kColdOrHot:
        return "Cold/Hot Fault";
      case TsStatus::kCool:
        return "Cool";
      case TsStatus::kWarm:
        return "Warm";
    }
    return "Unknown";
  }

  static const char *sysModeDescription(SysMode mode) {
    switch (mode) {
      case SysMode::kAutoVinOrBattery:
        return "Auto (VIN/BAT)";
      case SysMode::kBatteryOnly:
        return "Battery-only";
      case SysMode::kSysOffHighZ:
        return "SYS off (Hi-Z)";
      case SysMode::kSysOffPulldown:
        return "SYS off (Pulldown)";
    }
    return "Unknown";
  }

  static float decodeBatteryRegVoltage(uint8_t vbatCtrl) {
    const uint8_t code = vbatCtrl & 0x7F;
    return kVbatBaseVolts + static_cast<float>(code) * kVbatStepVolts;
  }

  static float decodeFastChargeCurrent(uint8_t ichgCtrl) {
    const uint8_t code = ichgCtrl & 0x7F;
    if (code <= kSlowChargeCodeMax) {
      return 5.0f + static_cast<float>(code);
    }
    const uint8_t fastCode = static_cast<uint8_t>(code - kFastChargeCodeOffset);
    return 40.0f + static_cast<float>(fastCode) * 10.0f;
  }

  static float decodeInputCurrentLimit(uint8_t tmrIlim) {
    const uint8_t code = tmrIlim & kTmrIlimIlimMask;
    switch (code) {
      case 0b000:
        return 50.0f;
      case 0b001:
        return 100.0f;
      case 0b010:
        return 200.0f;
      case 0b011:
        return 300.0f;
      case 0b100:
        return 400.0f;
      case 0b101:
        return 500.0f;
      case 0b110:
        return 665.0f;
      case 0b111:
        return 1050.0f;
    }
    return 0.0f;
  }

 private:
  static constexpr float kVbatBaseVolts = 3.5f;
  static constexpr float kVbatStepVolts = 0.01f;
  static constexpr uint8_t kVbatMaxCode = 115;  // 4.65 V target.

  static constexpr uint8_t kSlowChargeCodeMax = 30;
  static constexpr uint8_t kFastChargeCodeOffset = 31;

  static constexpr uint8_t kIchgDisableMask = 0x80;
  static constexpr uint8_t kIcCtrlWatchdogMask = 0x03;
  static constexpr uint8_t kSysRegWatchdog15sMask = 0x02;

  static constexpr uint8_t kSysModeShift = 2;
  static constexpr uint8_t kSysModeMask = static_cast<uint8_t>(0x03 << kSysModeShift);

  static constexpr uint8_t kShipActionShift = 5;
  static constexpr uint8_t kShipActionMask = static_cast<uint8_t>(0x03 << kShipActionShift);
  static constexpr uint8_t kShipRegisterResetMask = 0x80;

  static constexpr uint8_t kTmrIlimIlimMask = 0x07;

  Stat0Flags decodeStat0(uint8_t value) const {
    Stat0Flags flags;
    flags.tsOpen = (value & 0x80) != 0;
    flags.chargeState =
        static_cast<ChargeState>((value >> 5) & 0x03);
    flags.ilimActive = (value & 0x10) != 0;
    flags.vdppmActive = (value & 0x08) != 0;
    flags.vindpmActive = (value & 0x04) != 0;
    flags.thermalRegulation = (value & 0x02) != 0;
    flags.vinPowerGood = (value & 0x01) != 0;
    return flags;
  }

  Stat1Flags decodeStat1(uint8_t value) const {
    Stat1Flags flags;
    flags.vinOvp = (value & 0x80) != 0;
    flags.batteryUvlo = (value & 0x40) != 0;
    flags.tsStatus = static_cast<TsStatus>((value >> 3) & 0x03);
    flags.safetyTimerFault = (value & 0x04) != 0;
    flags.wake1 = (value & 0x02) != 0;
    flags.wake2 = (value & 0x01) != 0;
    return flags;
  }

  FaultFlags decodeFaults(uint8_t value) const {
    FaultFlags faults;
    faults.tsFault = (value & 0x80) != 0;
    faults.ilimFault = (value & 0x40) != 0;
    faults.vdppmFault = (value & 0x20) != 0;
    faults.vindpmFault = (value & 0x10) != 0;
    faults.thermalRegulationFlag = (value & 0x08) != 0;
    faults.vinOvpFault = (value & 0x04) != 0;
    faults.buvloFault = (value & 0x02) != 0;
    faults.batOcpFault = (value & 0x01) != 0;
    return faults;
  }

  SysMode decodeSysMode(uint8_t sysReg) const {
    return static_cast<SysMode>((sysReg & kSysModeMask) >> kSysModeShift);
  }

  static uint8_t encodeBatteryRegVoltage(float volts) {
    if (volts < kVbatBaseVolts) {
      volts = kVbatBaseVolts;
    }
    if (volts > (kVbatBaseVolts + kVbatStepVolts * kVbatMaxCode)) {
      volts = kVbatBaseVolts + kVbatStepVolts * kVbatMaxCode;
    }
    const float code = (volts - kVbatBaseVolts) / kVbatStepVolts;
    return static_cast<uint8_t>(roundf(code)) & 0x7F;
  }

  static uint8_t encodeFastChargeCurrent(float milliAmps) {
    if (milliAmps < 5.0f) {
      milliAmps = 5.0f;
    }
    if (milliAmps > 1000.0f) {
      milliAmps = 1000.0f;
    }
    if (milliAmps <= 35.0f) {
      return static_cast<uint8_t>(roundf(milliAmps - 5.0f)) & 0x7F;
    }
    const float delta = (milliAmps - 40.0f) / 10.0f;
    return static_cast<uint8_t>(roundf(delta) + kFastChargeCodeOffset) & 0x7F;
  }

  static uint8_t encodeInputCurrentLimit(float milliAmps) {
    if (milliAmps <= 50.0f) {
      return 0b000;
    }
    if (milliAmps <= 100.0f) {
      return 0b001;
    }
    if (milliAmps <= 200.0f) {
      return 0b010;
    }
    if (milliAmps <= 300.0f) {
      return 0b011;
    }
    if (milliAmps <= 400.0f) {
      return 0b100;
    }
    if (milliAmps <= 500.0f) {
      return 0b101;
    }
    if (milliAmps <= 665.0f) {
      return 0b110;
    }
    return 0b111;
  }

  TwoWire *wire_ = nullptr;
  uint8_t address_ = kDefaultI2cAddress;
};
