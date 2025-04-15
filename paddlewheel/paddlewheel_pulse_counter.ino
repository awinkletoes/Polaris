#include "driver/pcnt.h"

// Set up the pulse counter pin
const int pulsePin = 12;  // GPIO pin for pulse input

// Set up the pulse counter
pcnt_unit_t pcnt_unit = PCNT_UNIT_0;  // Choose the counter unit (PCNT0, PCNT1, etc.)
pcnt_channel_t pcnt_channel = PCNT_CHANNEL_0;  // Choose the channel (PCNT_CHANNEL_0 or PCNT_CHANNEL_1)

unsigned long lastMillis = 0;  // Variable to store the last time we reset the counter
const unsigned long interval = 10000;  // 10 seconds in milliseconds
float water_speed; //define water speed as float variable

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
  pcnt_config.neg_mode = PCNT_COUNT_INC;       // Also count falling edges
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

  //one rotation of the wheel is 6 pulses (it counts one paddle as either a falling or rising edge, so one rotation has 6 - verified)
  //circumference of the wheel = 11.624cm
  water_speed = (pulseCount/6)*11.624; //convert rotations into distance traveled (cm)
  water_speed = water_speed/10; //convert with time into cm/s
  water_speed = water_speed/51.444; //convert from cm/s to knots

  // Check if 60 seconds have passed
  if (millis() - lastMillis >= interval) {
    // Print the pulse count at the 60-second mark
    //Serial.print("Pulse Count: ");
    //Serial.println(pulseCount);

    //Print the water speed at the 60-second mark
    Serial.print("Water Speed (knots): ");
    Serial.println(water_speed, 3);

    // Reset the pulse counter
    pcnt_counter_clear(pcnt_unit);

    // Update the lastMillis time
    lastMillis = millis();  
  }

  // You can put a small delay here to prevent overloading the serial monitor
  delay(1000);  // Delay for readability (1 second, adjust as needed)
}