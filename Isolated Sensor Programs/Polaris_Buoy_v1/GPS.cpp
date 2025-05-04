#include <TinyGPS++.h>

static void smartDelay(TinyGPSPlus gps, unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available()) //changed to serial1
      gps.encode(Serial1.read()); //changed to serial1
  } while (millis() - start < ms);
}

static void showGPS(TinyGPSPlus gps)
{
  Serial.println();
  Serial.print("Latitude: ");
  Serial.println(gps.location.lat(), 8);//prints google maps latitude value
  Serial.print("Longitude: ");
  Serial.println(gps.location.lng(), 8);//prints google maps longitude value

  smartDelay(gps, 5000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}


