#include <RadioLib.h>
#include "LoRaBoards.h"
#include "utilities.h"

///////////////// LoRa Configuration ////////////////////
// SX1262 Setup - Match your pinout
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

#define CONFIG_RADIO_FREQUENCY      915.0
#define CONFIG_RADIO_BW             500.0
#define CONFIG_RADIO_SF             12
#define CONFIG_RADIO_OUTPUT_POWER   22
#define CONFIG_RADIO_RX_BOOSTED     true

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
  radio.setFrequency(CONFIG_RADIO_FREQUENCY);
  radio.setBandwidth(CONFIG_RADIO_BW);
  radio.setSpreadingFactor(CONFIG_RADIO_SF);
  radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER);
  radio.setRxBoostedGainMode(CONFIG_RADIO_RX_BOOSTED);
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
  Serial.printf("[SX1262] Starting receive mode... [%fMHz SF%d BW%.1f]:\n", CONFIG_RADIO_FREQUENCY, CONFIG_RADIO_SF, CONFIG_RADIO_BW);
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

    String rssi;
    String snr;
    String frequencyError;

    // Read the received data
    rssi = String(radio.getRSSI()) + "dBm";
    snr = String(radio.getSNR()) + "dB";
    frequencyError = String(radio.getFrequencyError()) + "Hz";
    String str;
    int state = radio.readData(str);

    Serial.printf("[LoRa] Received data [%fMHz SF%d BW%.1f]:\n", CONFIG_RADIO_FREQUENCY, CONFIG_RADIO_SF, CONFIG_RADIO_BW);
    Serial.println(str);

    //print RSSI
    Serial.print(F("Radio RSSI:\t\t"));
    Serial.println(rssi);

    // print SNR (Signal-to-Noise Ratio)
    Serial.print(F("Radio SNR:\t\t"));
    Serial.println(snr);

    Serial.print(F("Frequency error:\t"));
    Serial.println(frequencyError);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("[LoRa] Receive passed CRC");
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[LoRa] CRC error!"));
    } else {
      Serial.print("[LoRa] Receive failed, code ");
      Serial.println(state);
    }

    // Restart receiver
    radio.startReceive();
  }
}
