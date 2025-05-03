#include <Arduino.h>
#include <ArduinoLog.h>

#include "rtl-433-receiver.h"
#include "ble-server.h"

#include "rtl-433-receiver.h"
#include <HardwareSerial.h>

HardwareSerial SerialUART(1);  // UART1 for external TX/RX (GPIO4 and GPIO25)

// Create a class that logs to both Serial and SerialUART
class DualStream : public Stream {
public:
    Stream &s1;
    Stream &s2;

    DualStream(Stream &stream1, Stream &stream2) : s1(stream1), s2(stream2) {}

    int available() override { return s1.available(); }
    int read() override { return s1.read(); }
    int peek() override { return s1.peek(); }

    void flush() override {
        s1.flush();
        s2.flush();
    }

    size_t write(uint8_t c) override {
        s1.write(c);
        return s2.write(c);
    }
};

DualStream dualLog(Serial, SerialUART);  // Send logs to both USB and UART

void setup() {
    delay(2000);

    Serial.begin(115200);                       // USB Serial
    SerialUART.begin(115200, SERIAL_8N1, 25, 4); // UART TX=GPIO4, RX=GPIO25

#ifndef LOG_LEVEL
    #define LOG_LEVEL LOG_LEVEL_NOTICE
#endif

    Log.begin(LOG_LEVEL, &dualLog);  // Use our custom dual-output stream

    Log.notice(F(" " CR));
    Log.notice(F("****** setup ******" CR));
    rtl_433_receiver_init();
    ble_server_init();
    Log.notice(F("****** setup complete ******" CR));
}

void loop() {
    rtl_433_receiver_loop();
}
