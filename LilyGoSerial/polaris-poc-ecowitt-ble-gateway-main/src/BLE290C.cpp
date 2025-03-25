#include "sdkconfig.h"
#if defined(CONFIG_BLUEDROID_ENABLED)

#include "BLE290C.h"


BLE290C::BLE290C() : BLEDescriptor(BLEUUID((uint16_t) 0x290C)) {
    m_data.m_flags = 0;
    m_data.m_sampling_function = SAMPLING_UNSPECIFIED;
    m_data.m_measurement_period = MEASUREMENT_PERIOD_NOT_IN_USE;
    m_data.m_measurement_period_mso = 0;
    m_data.m_update_interval = UPDATE_INTERVAL_NOT_IN_USE;
    m_data.m_update_interval_mso = 0;
    m_data.m_application = APPLICATION_UNSPECIFIED;
    m_data.m_measurement_uncertainty = UNCERTAINTY_NA;
    setValue((uint8_t *) &m_data, sizeof(m_data));
} // BLE2902


void BLE290C::setSamplingFunction(uint8_t sampling_function) {
    m_data.m_sampling_function = sampling_function;
    setValue((uint8_t *) &m_data, sizeof(m_data));
}

void BLE290C::setMeasurementPeriod(uint32_t measurement_period) {
    m_data.m_measurement_period = measurement_period & 0xffff;
    m_data.m_measurement_period_mso = measurement_period >> 16;
    setValue((uint8_t *) &m_data, sizeof(m_data));
}

void BLE290C::setUpdateInterval(uint32_t update_interval) {
    m_data.m_update_interval = update_interval & 0xffff;
    m_data.m_update_interval_mso = update_interval >> 16;
    setValue((uint8_t *) &m_data, sizeof(m_data));
}

void BLE290C::setApplication(uint8_t application) {
    m_data.m_application = application;
    setValue((uint8_t *) &m_data, sizeof(m_data));
}

void BLE290C::setMeasurementUncertainty(uint8_t measurement_uncertainty) {
    m_data.m_measurement_uncertainty = measurement_uncertainty;
    setValue((uint8_t *) &m_data, sizeof(m_data));
}

#endif
