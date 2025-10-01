/*
 * Simple test sketch for Sharp Memory Display on AirGradient hardware
 * Tests basic display functionality with the pins configured in AgSharpDisplay.h
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

// Use the same pins as defined in AgSharpDisplay.h
#define SHARP_SCK 5
#define SHARP_MOSI 4
#define SHARP_SS 3

// Create display instance for 400x240 resolution with 4MHz SPI speed
// The last parameter (4000000) sets the SPI clock speed
Adafruit_SharpMem display(SHARP_SCK, SHARP_MOSI, SHARP_SS, 400, 240, 4000000);

#define BLACK 0
#define WHITE 1

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("========================================");
  Serial.println("Sharp Memory Display Test");
  Serial.println("========================================");
  Serial.println("Display: 400x240");
  Serial.println("SCK:  GPIO " + String(SHARP_SCK));
  Serial.println("MOSI: GPIO " + String(SHARP_MOSI));
  Serial.println("SS:   GPIO " + String(SHARP_SS));
  Serial.println("========================================");
  
  // Initialize display
  Serial.println("Initializing display...");
  if (!display.begin()) {
    Serial.println("ERROR: Display initialization failed!");
    Serial.println("Check connections:");
    Serial.println("  - VDD to 3.3V or 5V");
    Serial.println("  - GND to GND");
    Serial.println("  - SCK to GPIO 5");
    Serial.println("  - SI (MOSI) to GPIO 4");
    Serial.println("  - CS (SS) to GPIO 3");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("Display initialized successfully!");
  
  // Clear display
  display.clearDisplay();
  display.refresh();
  Serial.println("Display cleared");
  delay(1000);
  
  // Test 1: Single pixel
  Serial.println("\nTest 1: Drawing single pixel at (10,10)");
  display.drawPixel(10, 10, BLACK);
  display.refresh();
  delay(2000);
  display.clearDisplay();
  
  // Test 2: Diagonal line
  Serial.println("Test 2: Drawing diagonal line");
  display.drawLine(0, 0, display.width()-1, display.height()-1, BLACK);
  display.refresh();
  delay(2000);
  display.clearDisplay();
  
  // Test 3: Rectangle
  Serial.println("Test 3: Drawing rectangle");
  display.drawRect(10, 10, 100, 60, BLACK);
  display.refresh();
  delay(2000);
  display.clearDisplay();
  
  // Test 4: Filled rectangle
  Serial.println("Test 4: Drawing filled rectangle");
  display.fillRect(50, 50, 100, 60, BLACK);
  display.refresh();
  delay(2000);
  display.clearDisplay();
  
  // Test 5: Circle
  Serial.println("Test 5: Drawing circle");
  display.drawCircle(display.width()/2, display.height()/2, 50, BLACK);
  display.refresh();
  delay(2000);
  display.clearDisplay();
  
  // Test 6: Text
  Serial.println("Test 6: Drawing text");
  display.setTextSize(2);
  display.setTextColor(BLACK);
  display.setCursor(10, 10);
  display.println("Sharp Display");
  display.setCursor(10, 40);
  display.println("400x240 Test");
  display.setCursor(10, 70);
  display.println("AirGradient");
  display.refresh();
  delay(3000);
  display.clearDisplay();
  
  // Test 7: Large text centered
  Serial.println("Test 7: Large centered text");
  display.setTextSize(4);
  display.setCursor(80, 100);
  display.print("SUCCESS!");
  display.refresh();
  delay(3000);
  
  Serial.println("\n========================================");
  Serial.println("All tests completed!");
  Serial.println("========================================");
}

void loop() {
  // Cycle through different screens
  
  // Screen 1: Simple message
  display.clearDisplay();
  display.setTextSize(3);
  display.setCursor(50, 100);
  display.print("Display Works!");
  display.refresh();
  delay(3000);
  
  // Screen 2: Show display info
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 20);
  display.println("Sharp Memory Display");
  display.setCursor(10, 50);
  display.println("Resolution: 400x240");
  display.setCursor(10, 80);
  display.println("GPIO Pins:");
  display.setCursor(10, 110);
  display.print("  SCK:  ");
  display.println(SHARP_SCK);
  display.setCursor(10, 140);
  display.print("  MOSI: ");
  display.println(SHARP_MOSI);
  display.setCursor(10, 170);
  display.print("  SS:   ");
  display.println(SHARP_SS);
  display.refresh();
  delay(3000);
  
  // Screen 3: Animation
  display.clearDisplay();
  display.setTextSize(2);
  for (int i = 0; i < 5; i++) {
    display.setCursor(10, 100);
    display.print("Count: ");
    display.println(i);
    display.refresh();
    delay(500);
    display.fillRect(10, 100, 200, 30, WHITE);
  }
  
  Serial.println("Loop iteration complete");
  delay(1000);
}
