Github for Polaris Remote Sailing Buoy.

Goal of the project is to transmit weather data from a remote buoy 2 miles offshore in Lake Michigan to the Milwaukee Sailing Center

Sensors being used:

IMU (BNO055) 
PoC: IMU 
https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview
Use: 
  - Wave height measurement
  - Obtain z-axis movement
  - Double integrate for position data over time
  - I2C connection to LilyGo 1262

Anemometer (Ecowitt GW1103)
PoC: LilyGoSerial
https://www.ecowitt.com/shop/goodsDetail/126
Use: 
  - Wind speed and direction
  - Humidity
  - Using LoRa w/1276 chip to create BLE server and obtain rtl_433 data
  - Communicate to LoRa w/1262 chip via UART with data

Paddle Wheel (Garmin Current Speed Paddle Wheel)
PoC: paddlewheel
Use:
  - Determine current speed near the buoy
  - Uses pulse counter on the LilyGo 1262

GPS:
PoC: TinyGPS_Example
Use: 
  - Redundancy for location of buoy
  - UART communication to LilyGo 1262

Receive/transmit as well as TinyGPS examples pulled from here:
https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/tree/master

