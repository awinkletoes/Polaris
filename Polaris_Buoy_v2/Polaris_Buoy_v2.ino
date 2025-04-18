/// Libraries ///
#include <Adafruit_BNO055.h>
#include "Buoy_Sensors.h"
#include "LoRaBoards.h"
#include "utilities.h"
#include <RadioLib.h>

////////////////////////////////////////////////////////
/////////// Initialization of variables ////////////////
////////////////////////////////////////////////////////

////////////LoRa//////////////////////////
// Configuration for SX1262
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
// Initialize the SX1262 module

// Transmission finished flag
volatile bool transmittedFlag = false;
volatile bool transmitting = false;

// Function called when transmission complete
void setFlag(void) {
  transmittedFlag = true;
}

///////////////// GPS ///////////////////////////////
static const uint32_t GPSBaud = 9600;

HardwareSerial GPS_UART(1);

#define GPS_RX 15 //GPS Receive
#define GPS_TX 16 //GPS Transmit

TinyGPSPlus gps; // The TinyGPSPlus object

///////////////// IMU ///////////////////////////////
double DEG_2_RAD = 0.01745329251;
double windDir = 0;
double trueWindDir = 0;

// --- Timing and Constants --- //
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;  // (100Hz)
uint16_t PRINT_RATE_MS = 4000;
uint16_t printCounter = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

///////////////// Anemometer ///////////////////////////////
HardwareSerial Anem_UART(2); // Use UART2 (ESP32-S3)

// Define GPIO TX and RX pins
#define Anem_RX 42
#define Anem_TX 46

///////////////// Paddle Wheel ///////////////////////////////
float currentWaterSpeed = 0;

void setup() {
  delay(1000);
  Serial.begin(115200);

  setupBoards();

  
  delay(1500);
  // GPS //
  GPS_UART.begin(GPSBaud, SERIAL_8N1, GPS_RX, GPS_TX); 
  // Anemometer //
  Anem_UART.begin(115200, SERIAL_8N1, Anem_TX, Anem_RX); // UART 2
  // IMU //
  Serial.println("UART Receiver Initialized...");
 // Wire.setPins(43, 44);
//  delay(100);
// Now try initializing the BNO055
if (!bno.begin()) {
  Serial.println("No BNO055 detected on Wire1... Check wiring or address!");
} else {
  Serial.println("BNO055 initialized successfully on Wire1!");
}
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("Sensor Initialized.");
  Serial.println("--------------------------------------------------");

  pinMode(BOOT_PIN, INPUT);

  if (!Anem_UART) {
    Serial.println("Anemometer UART failed to initialize!");
  }

  if (!GPS_UART) {
    Serial.println("GPS failed to initialize!");
  }

  // Initialize SX1262
  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin();

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // Set interrupt action for LoRa
  radio.setPacketSentAction(setFlag);
}

unsigned long lastPrintTime = 0;

void loop() {
  // 1. Read GPS (feed GPS data)
  while (GPS_UART.available()) {
    gps.encode(GPS_UART.read());
  }
  String gpsStr = getGPS(gps);  // From earlier optimized version

  // 2. Read water speed
  String speedStr = getWaterSpeedInfo();  // ~10s blocking pulse count

  // 3. Read wave height (also ~6s of samples)
  String waveStr = getWaveHeightInfo(bno);

  // 4. Read wind info
  String windStr = getWindInfo(Anem_UART, bno);  // Compact version

  // 5. Combine and send/report
  String finalReport = gpsStr + speedStr + waveStr + windStr;
  Serial.print(finalReport);

  Serial.println("[LoRa] Sending data...");
  int state = radio.transmit(finalReport);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[LoRa] Transmission successful!");
  } else {
    Serial.print("[LoRa] Failed to transmit. Code: ");
    Serial.println(state);
  }

  // Wait before sending again
  delay(5000);  // Adjust depending on your sampling time
}
