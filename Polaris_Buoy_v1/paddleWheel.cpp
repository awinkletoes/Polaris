#include "driver/pcnt.h"
#include "Arduino.h"

// Constants for pulse counting
const int pulsePin = 12;  // GPIO pin for pulse input
const float PADDLE_CIRCUMFERENCE_CM = 11.624;
const int PULSES_PER_ROTATION = 6;
const float CM_PER_SEC_TO_KNOTS = 1.0 / 51.444;
const unsigned long intervalMs = 10000;  // 10 seconds

static float readWaterSpeedPCNT() {
  // Configure the pulse counter
  pcnt_config_t pcnt_config;
  pcnt_config.pulse_gpio_num = pulsePin;
  pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_INC;
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;

  // Initialize PCNT unit
  pcnt_unit_config(&pcnt_config);
  pcnt_counter_clear(PCNT_UNIT_0);

  // Wait for 10 seconds and count pulses
  unsigned long startTime = millis();
  while (millis() - startTime < intervalMs) {
    // just waiting
    delay(10);
  }

  // Get the pulse count
  int16_t pulseCount = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &pulseCount);

  // Calculate speed
  float rotations = pulseCount / float(PULSES_PER_ROTATION);
  float distance_cm = rotations * PADDLE_CIRCUMFERENCE_CM;
  float speed_cm_per_s = distance_cm / (intervalMs / 1000.0);
  float speed_knots = speed_cm_per_s * CM_PER_SEC_TO_KNOTS;

  return speed_knots;
}
