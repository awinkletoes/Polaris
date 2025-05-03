#ifndef GPS_H
#define GPS_H

#include <Arduino.h>

#define GPS_UART_NR 1
#define GPS_BAUD 9600

#define GPS_RX 16 //GPS Receive
#define GPS_TX 15 //GPS Transmit

bool initializeGPS();
String getGPS();

#endif //GPS_H
