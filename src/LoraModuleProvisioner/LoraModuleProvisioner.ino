/*
 * LoraModuleProvisioner.ino
 *
 * Minimal console helper that simply bridges USB serial to the Ebyte LoRa
 * modem and prints four common provisioning commands. The operator copies each
 * command into the serial monitor to configure the modem manually.
 */

#include <Arduino.h>

#include "../LoRa-SenseNode-v2/config.h"

namespace cfg = SensorNodeConfig;

HardwareSerial loraSerial(1);

void printInstructions() {
  Serial.println();
  Serial.println("LoRa Module Manual Provisioning");
  Serial.println("--------------------------------");
  Serial.println("Copy each command below into the serial monitor (press Enter after each line):");
  Serial.println();
  Serial.println("1) Read DevEUI:");
  Serial.println("   AT+DEVEUI?");
  Serial.println();
  Serial.println("2) Program your JoinEUI (replace <JOIN_EUI> with 16 hex chars):");
  Serial.println("   AT+APPEUI=<JOIN_EUI>");
  Serial.println();
  Serial.println("3) Program your AppKey (replace <APP_KEY> with 32 hex chars):");
  Serial.println("   AT+APPKEY=<APP_KEY>");
  Serial.println();
  Serial.println("4) Attempt an OTAA join:");
  Serial.println("   AT+JOIN");
  Serial.println();
  Serial.println("After the modem reports +EVT:JOINED you can send a test payload, e.g.:");
  Serial.println("   AT+SEND=2:54455354");
  Serial.println();
  Serial.println("Console is now in passthrough mode. Type commands to talk directly to the modem.");
  Serial.println();
}

void setup() {
  pinMode(cfg::kPinLoraEnable, OUTPUT);
  digitalWrite(cfg::kPinLoraEnable, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  loraSerial.begin(cfg::kLoraBaud, SERIAL_8N1, cfg::kLoraRxPin, cfg::kLoraTxPin);
  delay(50);
  loraSerial.print("AT\r\n");  // Wake the modem UART.

  printInstructions();
}

void loop() {
  while (Serial.available() > 0) {
    loraSerial.write(Serial.read());
  }
  while (loraSerial.available() > 0) {
    Serial.write(loraSerial.read());
  }
}
