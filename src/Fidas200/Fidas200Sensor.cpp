#include "Fidas200Sensor.h"

// Constructor: Initializes the sensor object with the specified serial port
Fidas200Sensor::Fidas200Sensor(HardwareSerial *port) : serialPort(port) {}

/**
 * @brief Initializes the serial communication with specified baud rate.
 * This function configures the serial port for communication with the sensor.
 *
 * @param baudRate The baud rate for serial communication. Default is 115200.
 */
void Fidas200Sensor::begin(unsigned long baudRate) {
  serialPort->begin(baudRate, SERIAL_8N1);
}

/**
 * @brief Calculates the Block Check Character (BCC) for a given data buffer.
 * This method performs an XOR operation across all characters in the buffer.
 *
 * @param buffer The data buffer for which the BCC is calculated.
 * @return The calculated BCC value.
 */
char Fidas200Sensor::calculateBCC(const char *buffer) {
  int i = 0, bcc = 0;
  while (buffer[i] != 0) {
    bcc ^= (unsigned char)buffer[i++];
  }
  return bcc;  // Return the final BCC value
}

/**
 * @brief Verifies the Block Check Character (BCC) of a received message.
 * The method extracts the BCC from the received message and compares it to the
 * calculated BCC.
 *
 * @param message The message received from the sensor, including the BCC.
 * @return True if the BCC is valid, false otherwise.
 */
bool Fidas200Sensor::verifyBCC(const String &message) {
  int start = message.indexOf('<');
  int end = message.indexOf('>');
  if (start == -1 || end == -1 || end <= start + 1) {
    return false;  // Return false if the message format is invalid
  }

  // Extract the data within the < > for BCC calculation
  String data = message.substring(start, end + 1);
  char receivedBCC = strtol(message.substring(end + 1).c_str(), nullptr, 16);
  char calculatedBCC = calculateBCC(data.c_str());

  return receivedBCC ==
         calculatedBCC;  // Return true if BCC matches, false otherwise
}

/**
 * @brief Parses the sensor values from the received message and stores them in
 * member variables.
 *
 * This method extracts sensor readings for temperature (°C), humidity (%), PM1
 * concentration (µg/m³), PM2.5 concentration (µg/m³), PM10 concentration
 * (µg/m³), and particle count from the given message. The message is expected
 * to be formatted in the following way:
 *
 * <sendVal 40=value;41=value;58=value;54=value;59=value;60=value>
 *
 * where:
 *  - 40 represents temperature in °C
 *  - 41 represents humidity in %
 *  - 58 represents PM2.5 concentration in mg/m³
 *  - 54 represents PM1 concentration in mg/m³
 *  - 59 represents PM10 concentration in mg/m³
 *  - 60 represents the particle count
 *
 * Values for PM1, PM2.5, and PM10 are converted from mg/m³ to µg/m³.
 * If any of the expected values (temperature, humidity, or PM2.5) are missing,
 * the function will exit without updating any values.
 *
 * @param message The message containing the data values enclosed within '<' and
 * '>' characters.
 */
void Fidas200Sensor::parseValues(const String &message) {
  int start = message.indexOf('<');
  int end = message.indexOf('>');
  if (start == -1 || end == -1) {
    return;  // Exit if the message format is invalid
  }

  // Extract the data within < >
  String data = message.substring(start + 1, end);
  int tempIndex = data.indexOf("40=");
  int humidityIndex = data.indexOf("41=");
  int pm25Index = data.indexOf("58=");
  int pm1Index = data.indexOf("54=");
  int pm10Index = data.indexOf("59=");
  int pmPCountIndex = data.indexOf("60=");

  if (tempIndex == -1 || humidityIndex == -1 || pm25Index == -1) {
    return;  // Exit if any expected values are missing
  }

  // Update the stored values for temperature, humidity, and PM2.5
  temperature =
      data.substring(tempIndex + 3, data.indexOf(';', tempIndex)).toFloat();
  humidity = data.substring(humidityIndex + 3, data.indexOf(';', humidityIndex))
                 .toFloat();
  pm25 = data.substring(pm25Index + 3, data.indexOf(';', pm25Index)).toFloat() *
         1000;  // Convert from mg/m³ to µg/m³
  pm1 = data.substring(pm1Index + 3, data.indexOf(';', pm1Index)).toFloat() *
        1000;  // Convert from mg/m³ to µg/m³
  pm10 = data.substring(pm10Index + 3, data.indexOf(';', pm10Index)).toFloat() *
         1000;  // Convert from mg/m³ to µg/m³
  pmPCount = data.substring(pmPCountIndex + 3).toFloat();
}

/**
 * @brief Sends a command to the Fidas 200 sensor.
 * The command requests measurement values for temperature, humidity, and PM2.5.
 */
void Fidas200Sensor::sendCommand() {
  char buffer[50] = "<getVal 40; 41; 58; 54; 59; 60>";

  // Calculate BCC for the command data
  char bcc = calculateBCC(buffer);

  // Send the command to the sensor
  serialPort->print(buffer);
  serialPort->printf("%02X\r\n", bcc);  // Append BCC in hexadecimal format

  waitingForResponse = true;  // Set flag to indicate waiting for a response
  lastSendTime = millis();    // Record the time of the last command sent
}

/**
 * @brief Reads the response from the Fidas 200 sensor.
 * This method checks if the response is available, verifies the BCC, and parses
 * the values if valid.
 */
void Fidas200Sensor::readResponse() {
  if (serialPort->available()) {
    String response = serialPort->readStringUntil(
        '\n');  // Read the incoming response until newline
    waitingForResponse =
        false;  // Clear the waiting flag after receiving the response

    // Verify the BCC of the received response
    if (verifyBCC(response)) {
      parseValues(response);  // Parse and store the values if BCC is valid
      retryCount = 0;         // Reset retry counter on successful response
    } else {
      sendCommand();  // Retry sending the command if BCC verification fails
    }
  }
}

/**
 * @brief Handles the communication with the sensor.
 * This function sends a request to the sensor and then waits for the response.
 * If there is no response within 5 seconds, it retries sending the command.
 */
void Fidas200Sensor::handle() {
  // If waiting for a response, check if 5 seconds have passed since the last
  // command was sent
  if (waitingForResponse) {
    if (millis() - lastSendTime >= 5000) {
      // If 5 seconds have passed without a response, retry sending the command
      retryCount++;
      if (retryCount <= 3) {
        sendCommand();  // Retry sending the command
      } else {
        // If retry limit is reached, reset the retry counter
        retryCount = 0;
        waitingForResponse = false;
      }
    }
  } else {
    // If not waiting for a response, send a new command
    sendCommand();
  }

  // Read the response if available
  readResponse();
}

/**
 * @brief Returns the latest temperature value.
 * @return The temperature in °C.
 */
float Fidas200Sensor::getTemperature() { return temperature; }

/**
 * @brief Returns the latest humidity value.
 * @return The humidity in %.
 */
float Fidas200Sensor::getHumidity() { return humidity; }

/**
 * @brief Returns the latest air pressure value.
 * @return The air pressure in hPa.
 */
float Fidas200Sensor::getPressure() { return pressure; }

/**
 * @brief Returns the latest PM2.5 concentration value.
 * @return The PM2.5 concentration in µg/m³.
 */
float Fidas200Sensor::getPM25() { return pm25; }

/**
 * @brief Returns the latest PM1 concentration value.
 * @return The PM1 concentration in µg/m³.
 */
float Fidas200Sensor::getPM1() { return pm1; }

/**
 * @brief Returns the latest PM10 concentration value.
 * @return The PM10 concentration in µg/m³.
 */
float Fidas200Sensor::getPM10() { return pm10; }

/**
 * @brief Returns the latest particle count value.
 * @return The particle count.
 */
float Fidas200Sensor::getPCount() { return pmPCount; }
