# OneOpenAir with Sharp Memory Display (400x240)

This is a modified version of the OneOpenAir example that uses a Sharp Memory Display (400x240 resolution) instead of the standard OLED display.

## Hardware Requirements

- AirGradient ONE or Open Air board
- Sharp Memory Display (400x240) - e.g., LS027B7DH01A
- Proper SPI connections

## Display Connections

By default, the Sharp Display uses these pins (defined in `AgSharpDisplay.h`):

- **SCK (Clock)**: GPIO 14
- **MOSI (Data)**: GPIO 13  
- **SS (Chip Select)**: GPIO 15

You can modify these pin assignments in `src/AgSharpDisplay.h` to match your hardware setup.

## Features

- 400x240 resolution display (vs 128x64 OLED)
- Three-column dashboard layout optimized for wider screen
- All standard AirGradient features (CO2, PM2.5, VOC, NOx, Temperature, Humidity)
- Firmware update progress with graphical progress bar
- Same API as OledDisplay for easy integration

## Usage

1. **Adjust pin assignments** in `src/AgSharpDisplay.h` if needed:
   ```cpp
   static const uint8_t SHARP_SCK = 14;   // Your clock pin
   static const uint8_t SHARP_MOSI = 13;  // Your data pin
   static const uint8_t SHARP_SS = 15;    // Your chip select pin
   ```

2. **Connect your Sharp Memory Display** to the specified pins

3. **Compile and upload** this example to your device

4. The display will show the three-column dashboard with sensor readings

## Important Notes

- **Do NOT use this on standard ONE_INDOOR hardware** - it has an OLED display, not a Sharp display
- This example is for **custom builds** with Sharp Memory Display hardware
- The Sharp Memory Display requires different power and signal handling than OLED
- Make sure your display is the 400x240 variant

## Troubleshooting

### Device keeps rebooting
- Check that your hardware actually has a Sharp Memory Display connected
- Verify pin connections match the pin definitions
- Ensure SPI pins are not conflicting with other peripherals
- Check power supply to the display

### Display shows nothing
- Verify SPI connections
- Check that the display power (VDD) is connected
- Ensure the chip select pin is correctly defined
- Try adjusting display refresh() timing

### Compilation errors
- Make sure you have the Adafruit_GFX and Adafruit_SharpMem libraries installed
- Check that all paths in includes are correct

## License

Same as the main AirGradient project - CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
