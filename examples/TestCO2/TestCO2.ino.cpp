# 1 "C:\\Users\\santa\\AppData\\Local\\Temp\\tmp7s44kaoo"
#include <Arduino.h>
# 1 "C:/Users/santa/OneDrive/Dokumen/AirGradient/Git/Fidas-200-AirGradient/examples/TestCO2/TestCO2.ino"







#include <AirGradient.h>

#ifdef ESP8266
AirGradient ag = AirGradient(DIY_BASIC);
#else

AirGradient ag = AirGradient(OPEN_AIR_OUTDOOR);
#endif

void failedHandler(String msg);
void setup();
void loop();
#line 19 "C:/Users/santa/OneDrive/Dokumen/AirGradient/Git/Fidas-200-AirGradient/examples/TestCO2/TestCO2.ino"
void setup()
{
  Serial.begin(115200);


#ifdef ESP8266
  if (ag.s8.begin(&Serial) == false)
  {
#else
  if (ag.s8.begin(Serial0) == false)
  {
#endif
    failedHandler("SenseAir S8 init failed");
  }
}

void loop()
{
  int co2Ppm = ag.s8.getCo2();
  Serial.printf("CO2: %d\r\n", co2Ppm);
  delay(5000);
}

void failedHandler(String msg)
{
  while (true)
  {
    Serial.println(msg);
    delay(1000);
  }
}