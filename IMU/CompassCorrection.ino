#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Arduino_JSON.h>

HardwareSerial SerialUART(1); // Use UART1 (ESP32-S3)

// Define TX and RX pins (Receiver should match Sender's TX)
#define UART_RX_PIN 47
#define UART_TX_PIN 48

double DEG_2_RAD = 0.01745329251;
double windDir = 0;
double trueWindDir = 0;

// --- Timing and Constants ---
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;  // (100Hz)
uint16_t PRINT_RATE_MS = 4000;
uint16_t printCounter = 0;


// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("BNO055 and Anemometer Example");
  SerialUART.begin(115200, SERIAL_8N1, UART_TX_PIN, UART_RX_PIN); // UART

    Serial.println("UART Receiver Initialized...");
  
    // --- I2C Pin Configuration (IMPORTANT: Adjust for your board!) ---
  Wire.setPins(43, 44);
  // Wire.begin(); // Use this for standard Arduinos (Uno, Nano, Mega etc.)
  if (!bno.begin()) {
    Serial.println("No BNO055 detected... Check wiring or I2C ADDR!");
    while (1)
      ;
  }
  //bno.setMode(OPERATION_MODE_COMPASS);
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("Sensor Initialized.");
  Serial.println("--------------------------------------------------");

  pinMode(BOOT_PIN, INPUT);
}

void loop() {
  if (SerialUART.available()) {  // Check if data is available
    String received = SerialUART.readString(); // Read until newline
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
  // Read Sensor Data ---
  //imu::Quaternion quat = bno.getQuat();
  //imu::Vector linearAccelBody = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Rotate Acceleration Vector from Body Frame to World Frame ---
 // imu::Vector linearAccelWorld = linearAccelBody;
  //linearAccelWorld = quat.rotateVector(linearAccelWorld);

  sensors_event_t event;
  bno.getEvent(&event);

if (printCounter * BNO055_SAMPLERATE_DELAY_MS >= PRINT_RATE_MS) {
  //imu::Vector orientation = quat.toEuler();
  double yaw = 0;//(orientation.x() / DEG_2_RAD);

  yaw = event.orientation.x;
  Serial.print("yaw: ");
  Serial.print(yaw);

  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.printf(",CAL-M:%d", mag);

  Serial.println();

  trueWindDir = (windDir + yaw);

  if(trueWindDir > 360){
    trueWindDir = trueWindDir - 360;
  }

  Serial.print("True Wind Direction: ");
  Serial.print(trueWindDir);
  printCounter = 0;
} else {
  printCounter++;
}

}

double parseWindDirection(const String& message) {
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
