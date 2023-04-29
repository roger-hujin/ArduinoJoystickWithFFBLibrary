#include "Utils.h"

// log usb data
const char* REPORT_NAMES[] = {"",
                              "SetEffect",
                              "SetEnvelop",
                              "SetCondition",
                              "SetPeriodic",
                              "SetConstantForce",
                              "SetRampForce",
                              "SetCustomForceData",
                              "SetDownloadForceSample",
                              "",
                              "EffectOperation",
                              "BlockFree",
                              "DeviceControl",
                              "DeviceGain",
                              "SetCustomForce"};

void LogUsbData(uint8_t* data, uint16_t len) {
  if (len > 0) {
    Serial.print(millis());
    Serial.print(" ");
    if (data[0] < 15) {
      Serial.print(REPORT_NAMES[data[0]]);
    } else {
      Serial.print(data[0], HEX);
    }
    Serial.print(": ");
    for (uint16_t i = 0; i < len; i++) {
      Serial.print(data[i], HEX);
      Serial.print(",");
    }
    Serial.println("");
  }
}
