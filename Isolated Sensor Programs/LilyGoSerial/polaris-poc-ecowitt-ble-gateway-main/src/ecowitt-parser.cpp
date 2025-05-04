#include "ecowitt-parser.h"

#include <ArduinoLog.h>
#include <ArduinoJson.h>
#include <ble-server.h>

void logJson(JsonDocument &doc) {
#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    char JSONmessageBuffer[measureJson(doc) + 1];
#else
    char JSONmessageBuffer[JSON_MSG_BUFFER];
#endif
    serializeJson(doc, JSONmessageBuffer, sizeof(JSONmessageBuffer));
#if defined(setBitrate) || defined(setFreqDev) || defined(setRxBW)
    Log.setShowLevel(false);
    Log.notice(F("."));
    Log.setShowLevel(true);
#else
    Log.notice(F("Received message : %s" CR), JSONmessageBuffer);
#endif
}

void parse_ecowitt(char *message) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);
    if (error) {
        Serial.println(F("Failed to parse JSON"));
        return;
    }
    logJson(doc);
    // TODO: Deal w/ missing / invalid data
    if (doc["model"] == "Fineoffset-WS80" || doc["model"] == "Fineoffset-WS90") {
        ble_update_true_wind_direction(doc["wind_dir_deg"]);
        ble_update_true_wind_speed_avg(doc["wind_avg_m_s"].as<double>());
        ble_update_true_wind_speed_max(doc["wind_max_m_s"].as<double>());
    }
}
