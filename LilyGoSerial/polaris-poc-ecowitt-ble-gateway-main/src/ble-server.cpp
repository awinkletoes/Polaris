#include "ble-server.h"

#include <BLEDevice.h>
#include <BLE2902.h>
#include <ArduinoLog.h>
#include <BLE290C.h>

// https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.service.environmental_sensing.xml
#define ENVIRONMENTAL_SERVICE_UUID (BLEUUID((uint16_t) 0x181A))
// https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.true_wind_speed.xml
#define TRUE_WIND_SPEED_CHARACTERISTIC_UUID (BLEUUID((uint16_t)0x2A70))
// https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.true_wind_direction.xml
#define TRUE_WIND_DIRECTION_CHARACTERISTIC_UUID (BLEUUID((uint16_t)0x2A71))

BLEServer *pServer = nullptr;
BLECharacteristic *pTrueWindDirection = nullptr;
BLECharacteristic *pTrueWindSpeedAvg = nullptr;
BLECharacteristic *pTrueWindSpeedMax = nullptr;

class CustomServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) override {
        pServer->startAdvertising();
    };

    void onDisconnect(BLEServer *pServer) override {
    }
};

void ble_server_init() {
    BLEDevice::init("ecowitt gateway");
    pServer = BLEDevice::createServer();

    pServer->setCallbacks(new CustomServerCallbacks());

    BLEService *pService = pServer->createService(ENVIRONMENTAL_SERVICE_UUID);

    auto averageMeasurementDescriptor = new BLE290C();
    averageMeasurementDescriptor->setSamplingFunction(BLE290C::SAMPLING_ARITHMETIC_MEAN);
    auto maximumMeasurementDescriptor = new BLE290C();
    maximumMeasurementDescriptor->setSamplingFunction(BLE290C::SAMPLING_MAXIMUM);
    auto instantaneousMeasurementDescriptor = new BLE290C();
    instantaneousMeasurementDescriptor->setSamplingFunction(BLE290C::SAMPLING_INSTANTANEOUS);

    pTrueWindSpeedAvg = pService->createCharacteristic(
        TRUE_WIND_SPEED_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTrueWindSpeedAvg->addDescriptor(new BLE2902());
    pTrueWindSpeedAvg->addDescriptor(averageMeasurementDescriptor);

    pTrueWindSpeedMax = pService->createCharacteristic(
        TRUE_WIND_SPEED_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTrueWindSpeedMax->addDescriptor(new BLE2902());
    pTrueWindSpeedMax->addDescriptor(maximumMeasurementDescriptor);

    pTrueWindDirection = pService->createCharacteristic(
        TRUE_WIND_DIRECTION_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTrueWindDirection->addDescriptor(new BLE2902());
    pTrueWindDirection->addDescriptor(instantaneousMeasurementDescriptor);

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(ENVIRONMENTAL_SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();

    Log.notice("Advertising started");
}

void ble_update_true_wind_direction(uint16_t direction) {
    Log.notice("Updating wind direction: %d\n", direction);

    direction = direction * 100;
    pTrueWindDirection->setValue(direction);
    pTrueWindDirection->notify();
}

void ble_update_true_wind_speed_avg(double speed) {
    Log.notice("Updating wind speed avg: %D\n", speed);

    uint16_t intSpeed = (uint16_t) (speed * 100);
    pTrueWindSpeedAvg->setValue(intSpeed);
    pTrueWindSpeedAvg->notify();
}

void ble_update_true_wind_speed_max(double speed) {
    Log.notice("Updating wind speed max: %D\n", speed);

    uint16_t intSpeed = (uint16_t) (speed * 100);
    pTrueWindSpeedMax->setValue(intSpeed);
    pTrueWindSpeedMax->notify();
}
