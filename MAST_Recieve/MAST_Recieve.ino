#include <RadioLib.h>
#include "LoRaBoards.h"
#include "utilities.h"

///////////////// LoRa Configuration ////////////////////
// SX1262 Setup - Match your pinout
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

volatile bool receivedFlag = false;
String receivedData = "";

// Callback when a packet is received
void setFlag(void) {
  receivedFlag = true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("Starting LoRa Receiver...");
  setupBoards();

  // Initialize the LoRa module
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[SX1262] Initialization successful!");
  } else {
    Serial.print("[SX1262] Initialization failed, code ");
    Serial.println(state);
    while (true);
  }

  // Set the function that gets called when a packet is received
  radio.setDio1Action(setFlag);

  // Start listening for packets
  Serial.println("[SX1262] Starting receive mode...");
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("[SX1262] Receive mode failed, code ");
    Serial.println(state);
    while (true);
  }
}

void loop() {
  if (receivedFlag) {
    receivedFlag = false;

    // Read the received data
    String str;
    int state = radio.readData(str);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("[LoRa] Received data:");
      Serial.println(str);
    } else {
      Serial.print("[LoRa] Receive failed, code ");
      Serial.println(state);
    }

    // Restart receiver
    radio.startReceive();
  }
}
