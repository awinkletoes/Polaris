#include "GPS.h"
#include <TinyGPS++.h>

HardwareSerial GPS_UART(GPS_UART_NR);
char gpsData[64];
portMUX_TYPE gpsMutex = portMUX_INITIALIZER_UNLOCKED;

void gpsReaderTask(void* pvParameters)
{
    const auto NO_DATA = "NaN,NaN,NaN\n";
    TinyGPSPlus gps;

    snprintf(gpsData, sizeof(gpsData), NO_DATA);
    Serial.println("GPS task started");

    for (;;)
    {
        while (GPS_UART.available())
        {
            int c = GPS_UART.read();
            if (gps.encode(c))
            {
                if (gps.location.isValid())
                {
                    portENTER_CRITICAL(&gpsMutex);
                    snprintf(gpsData, sizeof(gpsData), "%.6f,%.6f,%.2f\n",
                             gps.location.lat(), gps.location.lng(), gps.altitude.meters());
                    portEXIT_CRITICAL(&gpsMutex);
                }
                else
                {
                    portENTER_CRITICAL(&gpsMutex);
                    snprintf(gpsData, sizeof(gpsData), NO_DATA);
                    portEXIT_CRITICAL(&gpsMutex);
                }
            }
        }
        // Wait a little before trying to read more from the UART
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

bool initializeGPS()
{
    GPS_UART.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX); //Start GPS object

    if (!GPS_UART)
    {
        Serial.println("GPS UART failed to initialize!");
        return false;
    }

    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        gpsReaderTask,
        "GPS Data",
        4096,
        nullptr,
        1, // Priority of the task (1 is reasonable for non-critical)
        nullptr,
        1); // Core ID (0 or 1) - 1 is APP_CPU, recommended for Arduino
    if (taskCreated != pdPASS)
    {
        Serial.println("Failed to create GPS task");
        return false;
    }

    return true;
}

String getGPS()
{
    portENTER_CRITICAL(&gpsMutex);
    String gpsString = String(gpsData);
    portEXIT_CRITICAL(&gpsMutex);
    return gpsString;
}
