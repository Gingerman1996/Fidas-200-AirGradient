#include <Arduino.h>
#include <Wire.h>

#define I2C_SDA_PIN 7
#define I2C_SCL_PIN 6

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    delay(3000);
    Serial.println();
    Serial.println(F("I2C Scanner"));
    Serial.println(F("================"));

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
}

void loop() {
    byte error;
    uint8_t address;
    int devicesFound = 0;

    Serial.println(F("Scanning..."));

    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print(F("I2C device found at address 0x"));
            if (address < 16) {
                Serial.print('0');
            }
            Serial.print(address, HEX);
            Serial.println();
            devicesFound++;
        } else if (error == 4) {
            Serial.print(F("Unknown error at address 0x"));
            if (address < 16) {
                Serial.print('0');
            }
            Serial.println(address, HEX);
        }
        delay(5);
    }

    if (devicesFound == 0) {
        Serial.println(F("No I2C devices found"));
    }

    Serial.println();
    delay(2000);
}
