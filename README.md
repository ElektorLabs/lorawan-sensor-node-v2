# LoRa-Sensor-Node-v2

[![ESP32-S3](https://img.shields.io/badge/ESP32-S3-blue.svg)](https://www.espressif.com/en/products/socs/esp32-s3)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

A comprehensive ESP32-S3 based sensor node that validates GPS positioning, LoRaWAN connectivity, SD card logging, battery charging, and deep-sleep power management without requiring the Arduino IDE.

## 🚀 Features

- **Multi-Peripheral Validation**: Powers and tests GPS, LoRa modem, SD logger, and battery charger
- **Smart Power Management**: Configurable deep-sleep cycles with peripheral power control
- **LoRaWAN Integration**: OTAA join with synthetic telemetry transmission
- **GPS Cold Start Handling**: Waits for valid GPS fix before LoRaWAN operations
- **Battery Management**: BQ25186 charger monitoring and configuration
- **Comprehensive Logging**: Tagged UART output and SD card data logging
- **Status Indicators**: LED patterns for system health monitoring

## 🔧 Hardware Components

### Core Platform
- **MCU**: ESP32-S3 Development Module
- **GPS Module**: Ebyte EWM108-GN05 (Allystar-compatible GNSS receiver)
- **LoRa Module**: Ebyte LoRaWAN module with AT command interface
- **Battery Charger**: BQ25186 Li-Ion battery management IC
- **Storage**: MicroSD card for data logging

### Pin Configuration

| Component | Pin | Function |
|-----------|-----|----------|
| **Power Control** |
| I2C Enable | GPIO 38 | I2C bus power control |
| LoRa Enable | GPIO 6 | LoRa module power |
| GPS Enable | GPIO 7 | GPS module power |
| **Status LEDs** |
| Red LED | GPIO 48 | Error/fault indication |
| Green LED | GPIO 47 | System status |
| **I2C Bus** |
| SDA | GPIO 10 | I2C data line |
| SCL | GPIO 9 | I2C clock line |
| **SD Card (SPI)** |
| MOSI | GPIO 11 | SPI data out |
| SCK | GPIO 12 | SPI clock |
| MISO | GPIO 13 | SPI data in |
| CS | GPIO 8 | Chip select |
| **LoRa UART** |
| RX | GPIO 4 | LoRa module RX |
| TX | GPIO 5 | LoRa module TX |
| **GPS UART** |
| RX | GPIO 21 | GPS module RX |
| TX | GPIO 14 | GPS module TX |


## ⚙️ Configuration

All configuration is centralized in `config.h` using `constexpr` constants:

### Timing Parameters
```cpp
constexpr unsigned long kGpsColdStartMs = 60000;        // GPS cold start window
constexpr unsigned long kGpsLockTimeoutMs = 180000;     // GPS fix timeout
constexpr unsigned long kSensorAcquisitionMs = 15000;   // Sensor reading time
constexpr uint64_t kDeepSleepIntervalUs = 60ULL * 1000000ULL; // Sleep interval
constexpr unsigned long kLoraJoinRetryMs = 15000;       // LoRa join retry
```

### Power Management
```cpp
constexpr bool kEnableDeepSleep = true;     // Enable ESP32 deep sleep
constexpr bool kEnableStatusLeds = true;    // Enable LED indicators
constexpr float kChargeTargetCurrentMa = 520.0f;   // Battery charge current (5-1000 mA)
constexpr float kInputCurrentLimitMa = 665.0f;     // Input current limit (50-665 mA)
```

### LoRaWAN Credentials
```cpp
constexpr LoraCredentials kLoraCredentials{
    "xxxxxxxxxxxxxxxx",                    // Device EUI
    "0000000000000000",                    // Application EUI
    "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx",   // Application Key
};
```

### GPS Power Configuration
```cpp
constexpr AllystarGpsPowerConfig kGpsPowerConfig{
    true,    // Enable GPS sleep/wake commands
    90000,   // Sleep duration (90 seconds)
    0x01,    // Sleep action (deep sleep)
    0x11,    // Wake mode (start command)
};
```

## 🔄 Operation Flow

1. **System Wake**: ESP32 wakes from deep sleep, powers peripheral rails
2. **GPS Cold Start**: Waits for GPS module to acquire satellite fix
3. **LoRaWAN Join**: Performs OTAA join once GPS is ready (or timeout expires)
4. **Data Collection**: Gathers GPS coordinates and battery telemetry
5. **Transmission**: Sends telemetry via LoRaWAN uplink
6. **Logging**: Records data to SD card with timestamps
7. **Sleep Preparation**: Powers down GPS with Allystar sleep commands
8. **Deep Sleep**: ESP32 enters deep sleep for configured interval

## 📊 Serial Output

The firmware provides tagged serial output at 115200 baud:

```
[Pwr ] Wake reason: Timer
[GPS ] Powering GPS module...
[LoRa] Powering LoRa module...
[GPS ] NMEA: $GNGGA,123456.00,4012.34567,N,07412.34567,W,1,08,1.2,45.6,M,33.2,M,,*5A
[LoRa] AT+JOIN
[LoRa] Event: join confirmed
[LoRa] Sending telemetry: 4012.34567,-07412.34567,45.6,4.12,85
[Chg ] Battery: 4.12V, Charge: 85%, Temp: 25°C
[Pwr ] Entering deep sleep for 60 seconds
```

## 📁 File Structure

```
LoRa-SenseNode-v2/
├── LoRa-SenseNode-v2.ino    # Main firmware file
├── config.h                 # Configuration constants
├── Bq25186.h               # Battery charger interface
├── EbyteGpsParser.h        # GPS NMEA parser
├── LoraAtHelpers.h         # LoRaWAN AT command helpers
├── SdCardLogger.h          # SD card logging utilities
├── TaggedStream.h          # Serial output tagging
├── ProcessFlow.md          # Detailed process documentation
├── FirmwareOverview.md     # Architecture overview
└── docs/                   # Additional documentation
```

## 🔍 Troubleshooting

### LED Status Indicators
- **2 Red Blinks**: GPS fault or timeout
- **3 Red Blinks**: LoRa communication fault
- **Green Solid**: System operating normally
- **LEDs Off**: Status LEDs disabled in config

### Common Issues
1. **GPS Not Acquiring Fix**: Ensure clear sky view, check antenna connection
2. **LoRa Join Fails**: Verify credentials and gateway availability
3. **SD Card Errors**: Check card formatting (FAT32) and connections
4. **Power Issues**: Verify battery connection and charger configuration

### Debug Mode
Set `kEnableDebugSerial = true` and `kShowRawGps = true` in config.h for verbose output.

## 🛠️ Development

### Adding New Features
- Hardware tests: Create new sketches in `src/tests/<SketchName>/`
- Experiments: Use `src/claude-test/` for exploratory code
- Helper functions: Add to existing headers or create new ones as needed

### Testing Without Deep Sleep
Set `kEnableDeepSleep = false` in config.h to run continuous loops for bench testing.

## 📄 License

This project is licensed under the GNU License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request
