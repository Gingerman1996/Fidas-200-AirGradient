# Air Quality Measurement Project with Fidas 200

## Overview

This project is a fork of the [AirGradient Arduino](https://github.com/airgradienthq/arduino) repository, an open-source initiative developed by AirGradient. The purpose of this project is to use the I-9PSL board to read data from the Fidas 200 air quality measurement device. The data is read via a serial port and transmitted to the AirGradient dashboard. The parameters measured include PM1, PM2.5, PM10, Cn, Temperature, and Humidity.

## Hardware Used

The project utilizes the following hardware components:
- **I-9PSL Board**: The main controller used to interface with the Fidas 200. For more details, refer to the [I-9PSL board documentation](https://www.airgradient.com/documentation/one-v9/#schematics).
- **Fidas 200**: An air quality monitoring device capable of measuring PM1, PM2.5, PM10, Cn, temperature, and humidity. The connection to the Fidas 200 is made using the plug for the PMS, which includes a serial port. For more details, refer to the [Fidas 200 product page](https://www.palas.de/en/product/fidas200?save=accept&cookie[sessiontracking]=1).
- **RS232 to TTL Module**: Used for converting the RS232 signal from the Fidas 200 to TTL levels that the I-9PSL board can read. For more information, see the [RS232 to TTL module details](https://www.allnewstep.com/product/4929/%E0%B9%82%E0%B8%A1%E0%B8%94%E0%B8%B9%E0%B8%A5%E0%B9%81%E0%B8%9B%E0%B8%A5%E0%B8%87%E0%B8%AA%E0%B8%B1%E0%B8%8D%E0%B8%8D%E0%B8%B2%E0%B8%93-rs232-%E0%B9%80%E0%B8%9B%E0%B9%87%E0%B8%99-ttl-uart-module-rs232-sp3232-ttl-to-rs232).

## Purpose

The main goal of this project is to implement a system that can:
1. **Read data from the Fidas 200**: The I-9PSL board communicates with the Fidas 200 via the serial port, using an RS232 to TTL converter to interface between the devices.
2. **Transmit data to AirGradient dashboard**: Once the data (PM1, PM2.5, PM10, Cn, temperature, and humidity) is read from the Fidas 200, it is sent to the AirGradient dashboard for monitoring and visualization.

## How It Works

1. The I-9PSL board connects to the Fidas 200 using a serial connection via the RS232 to TTL module. The connection is made through the plug for the PMS, which provides access to the serial port.
2. The board continuously reads the air quality data (PM1, PM2.5, PM10, Cn), temperature, and humidity values from the Fidas 200.
3. The measured data is transmitted to the AirGradient dashboard for display and analysis.

## Code Modifications

The following modifications were made to adapt the code for use with the Fidas 200:

1. **Added Code for Reading Data from Fidas 200**:  
   New files `Fidas200Sensor.h` and `Fidas200Sensor.cpp` were added under the `src/Fidas200` directory. These files contain the code necessary to interface with the Fidas 200 air quality sensor.

2. **Modified the Example Code in `example/OneOpenAir`**:  
   - Removed the initialization and data reading code for sensors originally used with the I-9PSL board, as these sensors are not used in this project.
   - Added code to read data from the Fidas 200 instead.

   Here is the added function for reading data from the Fidas 200:

   ```cpp
   static void updateFidas(void) {
      fidasSensor.handle();

      // Retrieve the values and print them
      measurements.Temperature = fidasSensor.getTemperature();
      measurements.Humidity = fidasSensor.getHumidity();
      measurements.pm25_1 = fidasSensor.getPM25();
      measurements.pm01_1 = fidasSensor.getPM1();
      measurements.pm10_1 = fidasSensor.getPM10();
      measurements.pm03PCount_1 = fidasSensor.getPCount();

      Serial.printf(
            "Temperature: %.2f °C, Humidity: %d %%, PM2.5: %d µg/m³, PM1: %d µg/m³, "
            "PM10: %d µg/m³, Particle Count: %.2f\n",
            measurements.Temperature, measurements.Humidity, measurements.pm25_1,
            measurements.pm01_1, measurements.pm10_1, measurements.pm03PCount_1);
   }
   ```
   The Fidas 200 sensor data is updated using the above function. The retrieved values for temperature, humidity, and PM2.5 are stored in the `measurements`structure.
3. **Updated the Configuration for `OneOpenAir`:**
   The schedule for reading the Fidas 200 data was configured using the `AgSchedule`:
   ```cpp
   AgSchedule FidasSchedule(SENSOR_PM_UPDATE_INTERVAL, updateFidas);
   ```
   - An instance of `Fidas200Sensor` was created using:
   ```cpp
   Fidas200Sensor fidasSensor(&Serial0);
   ```
4. **Changes to `setup` and `loop` Functions:**
   - In the `setup` function, the sensor is initialized:
   ```cpp
   fidasSensor.begin(115200);
   ```
   - In the `loop` function, the schedule is run:
   ```cpp
   FidasSchedule.run();
   ```
5. **Modification to `AirGradient.h`:**
   Added an include statement to import the Fidas 200 sensor header:
   ```cpp
   #include "Fidas200/Fidas200Sensor.h"
   ```
## Repository Origin

This repository is a fork of the original [AirGradient Arduino repository](https://github.com/airgradienthq/arduino). The original project is maintained by AirGradient and provides open-source tools and libraries for air quality monitoring.