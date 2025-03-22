#include "driver/pcnt.h"

// Set up the pulse counter pin
const int pulsePin = 12;  // GPIO pin for pulse input

// Set up the pulse counter
pcnt_unit_t pcnt_unit = PCNT_UNIT_0;  // Choose the counter unit (PCNT0, PCNT1, etc.)
pcnt_channel_t pcnt_channel = PCNT_CHANNEL_0;  // Choose the channel (PCNT_CHANNEL_0 or PCNT_CHANNEL_1)

unsigned long lastMillis = 0;  // Variable to store the last time we reset the counter
const unsigned long interval = 60000;  // 60 seconds in milliseconds

void setup() {
  // Start serial communication
  Serial.begin(115200);
  delay(1000);

  // Configure the pulse counter
  pcnt_config_t pcnt_config;
  
  // Initialize the structure fields in the correct order
  pcnt_config.pulse_gpio_num = pulsePin;       // GPIO pin for pulse input
  pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;  // No control pin used
  pcnt_config.channel = PCNT_CHANNEL_0;        // Choose the channel (PCNT_CHANNEL_0 or PCNT_CHANNEL_1)
  pcnt_config.unit = PCNT_UNIT_0;              // Choose the counter unit (PCNT_UNIT_0 or PCNT_UNIT_1)
  pcnt_config.pos_mode = PCNT_COUNT_INC;       // Count rising edges
  pcnt_config.neg_mode = PCNT_COUNT_DIS;       // Do nothing on falling edges
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;     // Keep control mode as is
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;     // Keep control mode as is

  // Initialize the counter with the config
  pcnt_unit_config(&pcnt_config);
  
  // Set the counter to zero initially
  pcnt_counter_clear(pcnt_unit);

  lastMillis = millis();  // Initialize the lastMillis variable

}

void loop() {
  // Get the pulse count
  int16_t pulseCount = 0;
  pcnt_get_counter_value(pcnt_unit, &pulseCount);

  // Check if 60 seconds have passed
  if (millis() - lastMillis >= interval) {
    // Reset the pulse counter
    pcnt_counter_clear(pcnt_unit);
    lastMillis = millis();  // Update the lastMillis time
  }
  
  // Print the current pulse count
  Serial.print("Pulse Count: ");
  Serial.println(pulseCount);
  
  delay(60000);  // Delay for readability
}