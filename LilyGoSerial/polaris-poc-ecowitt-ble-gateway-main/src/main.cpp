#include <Arduino.h>
#include <ArduinoLog.h>

#include "rtl-433-receiver.h"
#include "ble-server.h"

#include "rtl-433-receiver.h"
#include <HardwareSerial.h>

HardwareSerial SerialUART(1);

void setup() {
    //Serial.begin(921600);
    SerialUART.begin(115200, SERIAL_8N1, 17, 18); // tx = 17, rx = 18
    delay(2000);
#ifndef LOG_LEVEL
    LOG_LEVEL LOG_LEVEL_NOTICE
#endif
    Log.begin(LOG_LEVEL, &SerialUART);
    Log.notice(F(" " CR));
    Log.notice(F("****** setup ******" CR));
    rtl_433_receiver_init();
    ble_server_init();
    Log.notice(F("****** setup complete ******" CR));
}

void loop() {
    rtl_433_receiver_loop();
}
