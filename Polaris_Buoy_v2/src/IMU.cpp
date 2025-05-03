#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

#include "buoy_data.pb.h"

#define EEPROM_CALIBRATION_START 0 // Calling IMU memory to get calibration data saved 

static bool parseWindData(const String& message, double& windDir, double& windSpeed) {
  // Clean the message
  String cleanedMessage = message;
  cleanedMessage.trim();
  cleanedMessage.replace("\r", "");
  cleanedMessage.replace("\n", "");

  int startIdx = cleanedMessage.indexOf("{");
  int endIdx = cleanedMessage.lastIndexOf("}");

  if (startIdx == -1 || endIdx == -1 || endIdx <= startIdx) {
    Serial.println("Error: No valid JSON object found in message.");
    return false;
  }

  String json = cleanedMessage.substring(startIdx, endIdx + 1);
  Serial.println("Extracted JSON: " + json);

  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.print("Parsing failed: ");
    Serial.println(error.f_str());
    return false;
  }

  //  Correct field names
  if (doc.containsKey("wind_dir_deg") &&
      (doc.containsKey("wind_avg_m_s") || doc.containsKey("wind_max_m_s"))) {

    windDir = doc["wind_dir_deg"];

    // Prefer wind_avg_m_s if available, otherwise fall back to wind_max_m_s
    if (doc.containsKey("wind_avg_m_s")) {
      windSpeed = doc["wind_avg_m_s"];
    } else {
      windSpeed = doc["wind_max_m_s"];
    }

    Serial.print("Parsed Wind Dir: ");
    Serial.println(windDir);
    Serial.print("Parsed Wind Speed: ");
    Serial.println(windSpeed);

    return true;
  }

  Serial.println("Missing required fields in the JSON message");
  return false;
}



// Collects wind info and returns a report String
static WindData getWindInfo(HardwareSerial &serial, Adafruit_BNO055 &bno) {
  double windDir = -1, windSpeed = -1, trueWindDir = -1;
  WindData windData = WindData_init_zero;

  unsigned long startTime = millis();

  // Wait for data to be available or timeout after 10 seconds
  while (!serial.available()) {
    if (millis() - startTime > 15000) {  // 15 seconds timeout
      Serial.println("Timeout: No wind data received.");
      return windData;
    }
    delay(10);  // Prevent CPU hogging
  }

  // Read the incoming data
  String received = serial.readString();

  // Parse the wind data if available
  if (parseWindData(received, windDir, windSpeed)) {
    sensors_event_t event;
    bno.getEvent(&event);
    double yaw = event.orientation.x;

    // Calculate true wind direction
    trueWindDir = fmod(windDir + yaw + 360, 360);  // Normalize to 0–360

    windData.has_windSpeedAverage_kmh_scaled100 = true;
    windData.windSpeedAverage_kmh_scaled100 = windSpeed * 3.6 * 100;
    windData.has_windSpeedDirection_degrees = true;
    windData.windSpeedDirection_degrees = trueWindDir;
  }

  // Return the wind data
  return windData;
}


/////EEPROM FUNCTIONS/////////

static void loadCalibration(Adafruit_BNO055 &bno) {
  uint8_t calibData[22];
  for (int i = 0; i < 22; i++) {
    calibData[i] = EEPROM.read(EEPROM_CALIBRATION_START + i);
  }

  bno.setSensorOffsets(calibData);
}

static bool isEEPROMDataAvailable() {
  for (int i = 0; i < 22; i++) {
    if (EEPROM.read(EEPROM_CALIBRATION_START + i) == 0xFF) {
      return false; // Data likely not present
    }
  }
  return true;
}



