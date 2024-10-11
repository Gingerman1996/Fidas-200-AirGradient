#ifndef FIDAS200SENSOR_H
#define FIDAS200SENSOR_H

#include <Arduino.h>

/**
 * @brief Fidas200Sensor class handles communication with the Fidas 200 sensor.
 * It provides methods for sending commands, reading responses, and parsing data.
 */
class Fidas200Sensor {
 private:
  HardwareSerial *serialPort;  // Serial port used for communication with the sensor
  bool waitingForResponse = false;  // Flag to check if waiting for a response from the sensor
  int retryCount = 0;  // Counter for retry attempts
  unsigned long lastSendTime = 0;  // Timestamp of the last command sent

  // Variables to store sensor data
  float temperature = 0.0;  // Temperature in °C
  float humidity = 0.0;     // Humidity in %
  float pressure = 0.0;     // Air pressure in hPa
  float pm25 = 0.0;         // PM2.5 concentration in µg/m³

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
   * Expected values include temperature, humidity, air pressure, and PM2.5 concentration.
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
  /**
   * @brief Constructor to initialize the Fidas200Sensor with a specific serial port.
   * @param port The HardwareSerial port used for communication with the sensor.
   */
  Fidas200Sensor(HardwareSerial *port);

  /**
   * @brief Initializes the serial communication with specified baud rate.
   * @param baudRate The baud rate for serial communication. Default is 115200.
   */
  void begin(unsigned long baudRate = 115200);

  /**
   * @brief Handles the communication with the sensor.
   * This function sends a request to the sensor and waits for the response.
   */
  void handle();

  /**
   * @brief Returns the latest temperature value.
   * @return The temperature in °C.
   */
  float getTemperature();

  /**
   * @brief Returns the latest humidity value.
   * @return The humidity in %.
   */
  float getHumidity();

  /**
   * @brief Returns the latest air pressure value.
   * @return The air pressure in hPa.
   */
  float getPressure();

  /**
   * @brief Returns the latest PM2.5 concentration value.
   * @return The PM2.5 concentration in µg/m³.
   */
  float getPM25();
};

#endif  // FIDAS200SENSOR_H
