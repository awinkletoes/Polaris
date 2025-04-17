#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Arduino_JSON.h>

static double parseWindDirection(const String& message) {
    int jsonStart = message.indexOf('{');
    if (jsonStart == -1) return -1; // No JSON found

    String json = message.substring(jsonStart); // Strip out prefix

    JSONVar data = JSON.parse(json);

    if (JSON.typeof(data) == "undefined") {
        Serial.println("Parsing failed!");
        return -1;
    }

    if (data.hasOwnProperty("wind_dir_deg")) {
        return double(data["wind_dir_deg"]);
    }

    return -1; // Key not found
}

static void receiveAnemometer(HardwareSerial &serial, double windDir){
  if (serial.available()) {  // Check if data is available
    String received = serial.readString(); // Read until newline
    Serial.print("Received: ");
    Serial.println(received); // Print to Serial Monitor

    windDir = parseWindDirection(received);
        if (windDir != -1) {
            Serial.print("Parsed wind direction: ");
            Serial.println(windDir, 2);  // print with 2 decimal places
        } else {
            Serial.println("Wind direction not found or parse error.");
        }
}
}

static void CompassCorrection(Adafruit_BNO055 bno, double windDir){
  double trueWindDir = 0;
  double yaw = 0; //(orientation.x() / DEG_2_RAD);

  sensors_event_t event;
  bno.getEvent(&event);

  yaw = event.orientation.x;
  
  trueWindDir = (windDir + yaw);

  if(trueWindDir > 360){
    trueWindDir = trueWindDir - 360;
  }

  Serial.print("True Wind Direction: ");
  Serial.print(trueWindDir);

}


