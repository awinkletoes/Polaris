#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <EEPROM.h>

// Create an instance of the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055();

// Define EEPROM address for calibration data storage
#define EEPROM_CALIBRATION_START 0

void setup() {
  Serial.begin(115200);
  Wire.begin(43,44);

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.print("Couldn't find the BNO055 sensor");
    while (1);
  }

  // Check if the calibration data is stored in EEPROM
  if (isEEPROMDataAvailable()) {
    // Restore the calibration data from EEPROM
    loadCalibration();
    Serial.println("Calibration loaded from EEPROM.");
  } else {
    Serial.println("No calibration data found, skipping load.");
  }

  // Now, you can perform your setup tasks
}

void loop() {
  // Read sensor calibration status
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  // Print calibration status
  Serial.print("Sys: ");
  Serial.print(sys);
  Serial.print(" Gyro: ");
  Serial.print(gyro);
  Serial.print(" Accel: ");
  Serial.print(accel);
  Serial.print(" Mag: ");
  Serial.println(mag);

  // Check if all three sensors are fully calibrated (calibration level 3)
  if (accel == 3 && gyro == 3 && mag == 3) {
    Serial.println("All sensors calibrated. Saving calibration data...");
    saveCalibration();
    delay(10000); // Optional: Delay to avoid saving repeatedly
  }

  // Your main program loop code goes here
  delay(1000);  // Main loop delay
}

// Function to save BNO055 calibration data to EEPROM
void saveCalibration() {
  uint8_t calibData[22];
  bno.getSensorOffsets(calibData);
  
  // Write calibration data to EEPROM
  for (int i = 0; i < 22; i++) {
    EEPROM.write(EEPROM_CALIBRATION_START + i, calibData[i]);
  }

  // Ensure data is written to EEPROM
  EEPROM.commit();
  Serial.println("Calibration data saved to EEPROM.");
}

// Function to load BNO055 calibration data from EEPROM
void loadCalibration() {
  uint8_t calibData[22];

  // Read calibration data from EEPROM
  for (int i = 0; i < 22; i++) {
    calibData[i] = EEPROM.read(EEPROM_CALIBRATION_START + i);
  }

  // Set the BNO055 calibration data
  bno.setSensorOffsets(calibData);
}

// Check if calibration data exists in EEPROM
bool isEEPROMDataAvailable() {
  for (int i = 0; i < 22; i++) {
    if (EEPROM.read(EEPROM_CALIBRATION_START + i) == 0xFF) {
      return false;  // Calibration data not found
    }
  }
  return true;  // Calibration data exists
}
