#include <TinyGPS++.h>
#include "buoy_data.pb.h"

static void smartDelay(TinyGPSPlus gps, unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available()) //changed to serial1
      gps.encode(Serial1.read()); //changed to serial1
  } while (millis() - start < ms);
}

static Telemetry getGPS(TinyGPSPlus& gps) {
  Telemetry telemetry = Telemetry_init_zero;
  if (gps.location.isValid()) {
    telemetry.has_gpsLatitude_degrees_scaled10000000 = true;
    telemetry.gpsLatitude_degrees_scaled10000000 = gps.location.lat() * 10000000;
    telemetry.has_gpsLongitude_degrees_scaled10000000 = true;
    telemetry.gpsLongitude_degrees_scaled10000000 = gps.location.lng() * 10000000;
    telemetry.has_gpsAltitude_meters_scaled100 = true;
    telemetry.gpsAltitude_meters_scaled100 = gps.altitude.meters() * 100;
  }
  return telemetry;
}
