/// Libraries ///
#include <Adafruit_BNO055.h>
#include "Buoy_Sensors.h"
#include "LoRaBoards.h"
#include <RadioLib.h>

////////////////////////////////////////////////////////
/////////// Initialization of variables ////////////////
////////////////////////////////////////////////////////

////////////LoRa//////////////////////////
SX1262 lora = new Module(SS, DIO1, RST, BUSY);

// transmission finished flag
volatile bool transmittedFlag = false;
volatile bool transmitting = false;

// function called when transmission complete
void setFlag(void) {
  transmittedFlag = true;
}

///////////////// GPS ///////////////////////////////
static const uint32_t GPSBaud = 9600;

HardwareSerial GPS_UART(1);

#define GPS_RX 15 //GPS Recieve
#define GPS_TX 16 //GPS Transmit

TinyGPSPlus gps; //the TinyGPSPlus object

///////////////// IMU ///////////////////////////////
double DEG_2_RAD = 0.01745329251;
double windDir = 0;
double trueWindDir = 0;

// --- Timing and Constants --- //
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;  // (100Hz)
uint16_t PRINT_RATE_MS = 4000;
uint16_t printCounter = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

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

  // GPS // 
  GPS_UART.begin(GPSBaud, SERIAL_8N1, GPS_RX, GPS_TX); 
  //Anemometer//
  Anem_UART.begin(115200, SERIAL_8N1, Anem_TX, Anem_RX); // UART 2
  // IMU //
  Serial.println("UART Receiver Initialized...");
  Wire.setPins(43, 44);
  if (!bno.begin()) {
    Serial.println("No BNO055 detected... Check wiring or I2C ADDR!");
    while (1)
      ;
  }
  bno.setMode(OPERATION_MODE_COMPASS);
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

// initialize SX1262
  Serial.print(F("[SX1262] Initializing ... "));
  int state = lora.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set interrupt action
  lora.setDio1Action(setFlag);


}

unsigned long lastPrintTime = 0;


void loop() {
  Serial.println("------GPS-----");
  showGPS(gps);
  Serial.println(" ");
  Serial.println("---------Anemometer--------");
  receiveAnemometer(Anem_UART, windDir);
  CompassCorrection(bno, windDir);
  Serial.println(" ");

  Serial.println("--------paddle wheel--------");
  // This line will block until the reading is done or timeout occurs
  float currentWaterSpeed = readWaterSpeedPCNT();  // pin 12
  Serial.print("Water Speed (knots): ");
  Serial.println(currentWaterSpeed, 3);
  Serial.println(" ");

  Serial.println("--------wave height--------");
  collectAndProcessWaveHeight(bno);
  Serial.println(" ");


}
