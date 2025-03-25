#ifndef COMPONENTS_CPP_UTILS_BLE290C_H_
#define COMPONENTS_CPP_UTILS_BLE290C_H_
#include "sdkconfig.h"
#if defined(CONFIG_BLUEDROID_ENABLED)

#include "BLEDescriptor.h"

struct BLE290C_Data {
  uint16_t m_flags; // Bitfield, reserved for future use
  uint8_t m_sampling_function; // Enumeration
  uint16_t m_measurement_period; // 1 second resolution
  uint8_t m_measurement_period_mso;
  uint16_t m_update_interval; // 1 second resolution
  uint8_t m_update_interval_mso;
  uint8_t m_application; // Enumeration
  uint8_t m_measurement_uncertainty; // Resolution 0.5 percent
} __attribute__((packed));

/**
 * @brief Descriptor for Characteristic Environmental Sensing Measurement.
 *
 * This is a convenience descriptor for the Characteristic Environmental Sensing Measurement which has a UUID of 0x290C.
 */
class BLE290C: public BLEDescriptor {
public:
	BLE290C();

	static const uint8_t SAMPLING_UNSPECIFIED   = 0;
	static const uint8_t SAMPLING_INSTANTANEOUS   = 1;
	static const uint8_t SAMPLING_ARITHMETIC_MEAN   = 2;
	static const uint8_t SAMPLING_RMS   = 3;
	static const uint8_t SAMPLING_MAXIMUM   = 4;
	static const uint8_t SAMPLING_MINIMUM   = 5;
	static const uint8_t SAMPLING_ACCUMULATED   = 6;
	static const uint8_t SAMPLING_COUNT   = 7;

	static const uint8_t MEASUREMENT_PERIOD_NOT_IN_USE   = 0;

    static const uint8_t UPDATE_INTERVAL_NOT_IN_USE      = 0;

    static const uint8_t APPLICATION_UNSPECIFIED = 0;
    static const uint8_t APPLICATION_AIR = 1;
    static const uint8_t APPLICATION_WATER = 2;
    static const uint8_t APPLICATION_BAROMETRIC = 3;

    static const uint8_t UNCERTAINTY_NA = 0xff;

    void setSamplingFunction(uint8_t sampling_function);
    void setMeasurementPeriod(uint32_t measurement_period);
    void setUpdateInterval(uint32_t update_interval);
    void setApplication(uint8_t application);
    void setMeasurementUncertainty(uint8_t measurement_uncertainty);

private:
	BLE290C_Data m_data;
}; // BLE290C

#endif /* CONFIG_BLUEDROID_ENABLED */
#endif /* COMPONENTS_CPP_UTILS_BLE290C_H_ */
