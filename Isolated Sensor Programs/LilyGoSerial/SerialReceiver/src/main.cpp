#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial SerialUART(1); // Use UART1 (ESP32-S3)

// Define TX and RX pins (Receiver should match Sender's TX)
#define UART_RX_PIN 47
#define UART_TX_PIN 48

void setup() {
    Serial.begin(115200); // USB Serial for Serial Monitor
    SerialUART.begin(115200, SERIAL_8N1, UART_TX_PIN, UART_RX_PIN); // UART

    Serial.println("UART Receiver Initialized...");
}

void loop() {
    if (SerialUART.available()) {  // Check if data is available
        String received = SerialUART.readStringUntil('\n'); // Read until newline
        Serial.print("Received: ");
        Serial.println(received); // Print to Serial Monitor
    }
}
