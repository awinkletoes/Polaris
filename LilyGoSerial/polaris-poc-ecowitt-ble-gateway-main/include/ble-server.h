#ifndef BLE_SERVER_H
#define BLE_SERVER_H
#include <cstdint>

void ble_server_init();

void ble_update_true_wind_direction(uint16_t direction);

void ble_update_true_wind_speed_avg(double speed);

void ble_update_true_wind_speed_max(double speed);

#endif //BLE_SERVER_H
