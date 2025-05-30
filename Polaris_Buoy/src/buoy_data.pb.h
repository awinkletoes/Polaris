/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.9.1 */

#ifndef PB_BUOY_DATA_PB_H_INCLUDED
#define PB_BUOY_DATA_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _WindData {
    bool has_windSpeedDirection_degrees;
    uint32_t windSpeedDirection_degrees;
    bool has_windSpeedAverage_kmh_scaled100;
    uint32_t windSpeedAverage_kmh_scaled100;
    bool has_windSpeedGust_kmh_scaled100;
    uint32_t windSpeedGust_kmh_scaled100;
    bool has_windSpeedLull_kmh_scaled100;
    uint32_t windSpeedLull_kmh_scaled100;
} WindData;

typedef struct _AirData {
    bool has_airTemperature_celsius_scaled10;
    int32_t airTemperature_celsius_scaled10;
    bool has_airRelativeHumidity_percent_scaled10;
    uint32_t airRelativeHumidity_percent_scaled10;
    bool has_airPressure_hpa_scaled100;
    uint32_t airPressure_hpa_scaled100;
} AirData;

typedef struct _CurrentData {
    bool has_surfaceCurrentDirection_degrees;
    uint32_t surfaceCurrentDirection_degrees;
    bool has_surfaceCurrentSpeed_kmh_scaled100;
    uint32_t surfaceCurrentSpeed_kmh_scaled100;
    bool has_surfaceTemperature_celsius_scaled10;
    int32_t surfaceTemperature_celsius_scaled10;
} CurrentData;

typedef struct _WaveData {
    bool has_dominantWavePeriod_seconds_scaled100;
    uint32_t dominantWavePeriod_seconds_scaled100;
    bool has_significantWaveHeight_meters_scaled100;
    uint32_t significantWaveHeight_meters_scaled100;
    bool has_meanWaveHeightHighestTenth_meters_scaled100;
    uint32_t meanWaveHeightHighestTenth_meters_scaled100;
    bool has_maximumWaveHeight_meters_scaled100;
    uint32_t maximumWaveHeight_meters_scaled100;
} WaveData;

typedef struct _Telemetry {
    bool has_gpsLatitude_degrees_scaled10000000;
    int32_t gpsLatitude_degrees_scaled10000000;
    bool has_gpsLongitude_degrees_scaled10000000;
    int32_t gpsLongitude_degrees_scaled10000000;
    bool has_gpsAltitude_meters_scaled100;
    uint32_t gpsAltitude_meters_scaled100;
} Telemetry;

typedef struct _BuoyData {
    bool has_windData;
    WindData windData;
    bool has_airData;
    AirData airData;
    bool has_currentData;
    CurrentData currentData;
    bool has_waveData;
    WaveData waveData;
    bool has_telemetry;
    Telemetry telemetry;
} BuoyData;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define WindData_init_default                    {false, 0, false, 0, false, 0, false, 0}
#define AirData_init_default                     {false, 0, false, 0, false, 0}
#define CurrentData_init_default                 {false, 0, false, 0, false, 0}
#define WaveData_init_default                    {false, 0, false, 0, false, 0, false, 0}
#define Telemetry_init_default                   {false, 0, false, 0, false, 0}
#define BuoyData_init_default                    {false, WindData_init_default, false, AirData_init_default, false, CurrentData_init_default, false, WaveData_init_default, false, Telemetry_init_default}
#define WindData_init_zero                       {false, 0, false, 0, false, 0, false, 0}
#define AirData_init_zero                        {false, 0, false, 0, false, 0}
#define CurrentData_init_zero                    {false, 0, false, 0, false, 0}
#define WaveData_init_zero                       {false, 0, false, 0, false, 0, false, 0}
#define Telemetry_init_zero                      {false, 0, false, 0, false, 0}
#define BuoyData_init_zero                       {false, WindData_init_zero, false, AirData_init_zero, false, CurrentData_init_zero, false, WaveData_init_zero, false, Telemetry_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define WindData_windSpeedDirection_degrees_tag  1
#define WindData_windSpeedAverage_kmh_scaled100_tag 2
#define WindData_windSpeedGust_kmh_scaled100_tag 3
#define WindData_windSpeedLull_kmh_scaled100_tag 4
#define AirData_airTemperature_celsius_scaled10_tag 1
#define AirData_airRelativeHumidity_percent_scaled10_tag 2
#define AirData_airPressure_hpa_scaled100_tag    3
#define CurrentData_surfaceCurrentDirection_degrees_tag 1
#define CurrentData_surfaceCurrentSpeed_kmh_scaled100_tag 2
#define CurrentData_surfaceTemperature_celsius_scaled10_tag 3
#define WaveData_dominantWavePeriod_seconds_scaled100_tag 1
#define WaveData_significantWaveHeight_meters_scaled100_tag 2
#define WaveData_meanWaveHeightHighestTenth_meters_scaled100_tag 3
#define WaveData_maximumWaveHeight_meters_scaled100_tag 4
#define Telemetry_gpsLatitude_degrees_scaled10000000_tag 1
#define Telemetry_gpsLongitude_degrees_scaled10000000_tag 2
#define Telemetry_gpsAltitude_meters_scaled100_tag 3
#define BuoyData_windData_tag                    1
#define BuoyData_airData_tag                     2
#define BuoyData_currentData_tag                 3
#define BuoyData_waveData_tag                    4
#define BuoyData_telemetry_tag                   5

/* Struct field encoding specification for nanopb */
#define WindData_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, UINT32,   windSpeedDirection_degrees,   1) \
X(a, STATIC,   OPTIONAL, UINT32,   windSpeedAverage_kmh_scaled100,   2) \
X(a, STATIC,   OPTIONAL, UINT32,   windSpeedGust_kmh_scaled100,   3) \
X(a, STATIC,   OPTIONAL, UINT32,   windSpeedLull_kmh_scaled100,   4)
#define WindData_CALLBACK NULL
#define WindData_DEFAULT NULL

#define AirData_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, SINT32,   airTemperature_celsius_scaled10,   1) \
X(a, STATIC,   OPTIONAL, UINT32,   airRelativeHumidity_percent_scaled10,   2) \
X(a, STATIC,   OPTIONAL, UINT32,   airPressure_hpa_scaled100,   3)
#define AirData_CALLBACK NULL
#define AirData_DEFAULT NULL

#define CurrentData_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, UINT32,   surfaceCurrentDirection_degrees,   1) \
X(a, STATIC,   OPTIONAL, UINT32,   surfaceCurrentSpeed_kmh_scaled100,   2) \
X(a, STATIC,   OPTIONAL, SINT32,   surfaceTemperature_celsius_scaled10,   3)
#define CurrentData_CALLBACK NULL
#define CurrentData_DEFAULT NULL

#define WaveData_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, UINT32,   dominantWavePeriod_seconds_scaled100,   1) \
X(a, STATIC,   OPTIONAL, UINT32,   significantWaveHeight_meters_scaled100,   2) \
X(a, STATIC,   OPTIONAL, UINT32,   meanWaveHeightHighestTenth_meters_scaled100,   3) \
X(a, STATIC,   OPTIONAL, UINT32,   maximumWaveHeight_meters_scaled100,   4)
#define WaveData_CALLBACK NULL
#define WaveData_DEFAULT NULL

#define Telemetry_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, SINT32,   gpsLatitude_degrees_scaled10000000,   1) \
X(a, STATIC,   OPTIONAL, SINT32,   gpsLongitude_degrees_scaled10000000,   2) \
X(a, STATIC,   OPTIONAL, UINT32,   gpsAltitude_meters_scaled100,   3)
#define Telemetry_CALLBACK NULL
#define Telemetry_DEFAULT NULL

#define BuoyData_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  windData,          1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  airData,           2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  currentData,       3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  waveData,          4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  telemetry,         5)
#define BuoyData_CALLBACK NULL
#define BuoyData_DEFAULT NULL
#define BuoyData_windData_MSGTYPE WindData
#define BuoyData_airData_MSGTYPE AirData
#define BuoyData_currentData_MSGTYPE CurrentData
#define BuoyData_waveData_MSGTYPE WaveData
#define BuoyData_telemetry_MSGTYPE Telemetry

extern const pb_msgdesc_t WindData_msg;
extern const pb_msgdesc_t AirData_msg;
extern const pb_msgdesc_t CurrentData_msg;
extern const pb_msgdesc_t WaveData_msg;
extern const pb_msgdesc_t Telemetry_msg;
extern const pb_msgdesc_t BuoyData_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define WindData_fields &WindData_msg
#define AirData_fields &AirData_msg
#define CurrentData_fields &CurrentData_msg
#define WaveData_fields &WaveData_msg
#define Telemetry_fields &Telemetry_msg
#define BuoyData_fields &BuoyData_msg

/* Maximum encoded size of messages (where known) */
#define AirData_size                             18
#define BUOY_DATA_PB_H_MAX_SIZE                  BuoyData_size
#define BuoyData_size                            112
#define CurrentData_size                         18
#define Telemetry_size                           18
#define WaveData_size                            24
#define WindData_size                            24

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
