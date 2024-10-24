#ifndef FIDAS200SENSOR_H
#define FIDAS200SENSOR_H

#include <Arduino.h>

/**
 * @brief Fidas200Sensor class handles communication with the Fidas 200 sensor.
 * It provides methods for sending commands, reading responses, and parsing
 * data.
 */
class Fidas200Sensor {
 private:
  HardwareSerial
      *serialPort;  // Serial port used for communication with the sensor
  bool waitingForResponse =
      false;  // Flag to check if waiting for a response from the sensor
  int retryCount = 0;              // Counter for retry attempts
  unsigned long lastSendTime = 0;  // Timestamp of the last command sent

  // Variables to store sensor data
  float temperature = 0.0;  // Temperature in °C
  float humidity = 0.0;     // Humidity in %
  float pressure = 0.0;     // Air pressure in hPa
  float pm25 = 0.0;         // PM2.5 concentration in µg/m³
  float pmPCount = 0.0;     // PM Cn P/cm³
  float pm1 = 0.0;          // PM1 concentration in µg/m³
  float pm10 = 0.0;         // PM10 concentration in µg/m³

  /**
   * @brief Calculates the Block Check Character (BCC) for a given data buffer.
   * @param buffer The data buffer for which the BCC is calculated.
   * @return The calculated BCC value.
   */
  char calculateBCC(const char *buffer);

  /**
   * @brief Verifies the BCC of a received message.
   * @param message The message received from the sensor.
   * @return True if the BCC is valid, false otherwise.
   */
  bool verifyBCC(const String &message);

  /**
   * @brief Parses the values from the received message and stores them.
   * Expected values include temperature, humidity, air pressure, and PM2.5
   * concentration.
   * @param message The message containing the data values.
   */
  void parseValues(const String &message);

  /**
   * @brief Sends a command to the Fidas 200 sensor.
   * The command requests specific measurement values.
   */
  void sendCommand();

  /**
   * @brief Reads the response from the Fidas 200 sensor.
   * If a valid response is received, it verifies the BCC and parses the values.
   */
  void readResponse();

 public:
  Fidas200Sensor(HardwareSerial *port);
  void begin(unsigned long baudRate = 115200);
  void handle();
  float getTemperature();
  float getHumidity();
  float getPressure();
  float getPM25();
  float getPM1();
  float getPM10();
  float getPCount();
};

#endif  // FIDAS200SENSOR_H
