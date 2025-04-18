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

static String getGPS(TinyGPSPlus& gps) {
  if (gps.location.isValid()) {
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    float alt = gps.altitude.meters();
    return String(lat, 6) + "," + String(lng, 6) + "," + String(alt, 2);
  } else {
    return "NaN,NaN,NaN";  // or optionally return an empty string ""
  }
}
