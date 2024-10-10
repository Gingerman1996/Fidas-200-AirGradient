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
 * The method extracts the BCC from the received message and compares it to the calculated BCC.
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

  return receivedBCC == calculatedBCC;  // Return true if BCC matches, false otherwise
}

/**
 * @brief Parses the values from the received message and stores them in member variables.
 * This method extracts temperature (°C), humidity (%), air pressure (hPa), and PM2.5 concentration (µg/m³) from the message.
 * 
 * @param message The message containing the data values, formatted as <sendVal 40=value;41=value;58=value>.
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
  int pmIndex = data.indexOf("58=");

  if (tempIndex == -1 || humidityIndex == -1 || pmIndex == -1) {
    return;  // Exit if any expected values are missing
  }

  // Update the stored values for temperature, humidity, and PM2.5
  temperature = data.substring(tempIndex + 3, data.indexOf(';', tempIndex)).toFloat();
  humidity = data.substring(humidityIndex + 3, data.indexOf(';', humidityIndex)).toFloat();
  pm25 = data.substring(pmIndex + 3).toFloat() * 1000;  // Convert from mg/m³ to µg/m³
}

/**
 * @brief Sends a command to the Fidas 200 sensor.
 * The command requests measurement values for temperature, humidity, and PM2.5.
 */
void Fidas200Sensor::sendCommand() {
  if (waitingForResponse) {
    return;  // Do not send a new command if still waiting for a response
  }

  char buffer[50] = "<getVal 40; 41; 58>";

  // Calculate BCC for the command data
  char bcc = calculateBCC(buffer);

  // Send the command to the sensor
  serialPort->print(buffer);
  serialPort->printf("%02X\r\n", bcc);  // Append BCC in hexadecimal format

  waitingForResponse = true;  // Set flag to indicate waiting for a response
}

/**
 * @brief Reads the response from the Fidas 200 sensor.
 * This method checks if the response is available, verifies the BCC, and parses the values if valid.
 */
void Fidas200Sensor::readResponse() {
  if (serialPort->available()) {
    String response = serialPort->readStringUntil('\n');  // Read the incoming response until newline
    waitingForResponse = false;  // Clear the waiting flag after receiving the response

    // Verify the BCC of the received response
    if (verifyBCC(response)) {
      parseValues(response);  // Parse and store the values if BCC is valid
    } else {
      sendCommand();  // Retry sending the command if BCC verification fails
    }
  }
}

/**
 * @brief Handles the communication with the sensor.
 * This function sends a request to the sensor and then waits for the response.
 */
void Fidas200Sensor::handle() {
  // Send the command to the sensor
  sendCommand();
  
  // Wait for a short period to allow the sensor to respond
  delay(100);

  // Read the response from the sensor
  readResponse();
}

/**
 * @brief Returns the latest temperature value.
 * @return The temperature in °C.
 */
float Fidas200Sensor::getTemperature() {
  return temperature;
}

/**
 * @brief Returns the latest humidity value.
 * @return The humidity in %.
 */
float Fidas200Sensor::getHumidity() {
  return humidity;
}

/**
 * @brief Returns the latest air pressure value.
 * @return The air pressure in hPa.
 */
float Fidas200Sensor::getPressure() {
  return pressure;
}

/**
 * @brief Returns the latest PM2.5 concentration value.
 * @return The PM2.5 concentration in µg/m³.
 */
float Fidas200Sensor::getPM25() {
  return pm25;
}
