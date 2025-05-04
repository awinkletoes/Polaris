#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

const uint32_t TIMER_FREQUENCY_HZ = 1000000;   // Timer frequency: 1MHz (1 tick = 1 microsecond)
const uint64_t TIMER_ALARM_BNO055_US = 10000;  // Timer alarm period: 10,000 us = 10 ms, e.g. 100Hz

hw_timer_t* timerBNO055 = NULL;
volatile SemaphoreHandle_t timerBNO055Semaphore = NULL;
TaskHandle_t deferredBNO055TaskHandle = NULL;

// --- World frame state variables ---
volatile double xPos = 0.0;  // Position in world X frame (e.g. meters)
volatile double yPos = 0.0;  // Position in world Y frame (e.g. meters)
volatile double zPos = 0.0;  // Position in world Z frame (e.g. meters)
volatile double xVel = 0.0;  // Velocity in world X frame (e.g. m/s)
volatile double yVel = 0.0;  // Velocity in world Y frame (e.g. m/s)
volatile double zVel = 0.0;  // Velocity in world Z frame (e.g. m/s)
volatile double xAcc = 0.0;  // Acceleration in world X frame (e.g. m/s2)
volatile double yAcc = 0.0;  // Acceleration in world Y frame (e.g. m/s2)
volatile double zAcc = 0.0;  // Acceleration in world Z frame (e.g. m/s2)

const double DEG_2_RAD = 0.01745329251;  // Degrees to Radians

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void IRAM_ATTR resetPosition() {
  xAcc = yAcc = zAcc = xVel = yVel = zVel = xPos = yPos = zPos = 0;
}

void IRAM_ATTR onBNO055Timer() {
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(timerBNO055Semaphore, &higherPriorityTaskWoken);
  if (higherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void deferredBNO055ProcessingTask(void* pvParameters) {
  Serial.println("BNO055 Data Acquisition Task started.");

  // Time delta in seconds, derived from sample rate
  unsigned long lastUpdateTime = micros();
  double dt = 0;

  // Infinite loop: Wait for signal, process, repeat
  for (;;) {
    // Wait indefinitely for the semaphore to be given by the ISR
    if (xSemaphoreTake(timerBNO055Semaphore, portMAX_DELAY) == pdTRUE) {

      // Read Sensor Data ---
      imu::Quaternion quat = bno.getQuat();
      imu::Vector linearAccelBody = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

      unsigned long tStart = micros();
      dt = (tStart - lastUpdateTime) / (double)TIMER_FREQUENCY_HZ;
      lastUpdateTime = tStart;

      // Rotate Acceleration Vector from Body Frame to World Frame ---
      imu::Vector linearAccelWorld = linearAccelBody;
      linearAccelWorld = quat.rotateVector(linearAccelWorld);
      xAcc = linearAccelWorld.x();
      yAcc = linearAccelWorld.y();
      zAcc = linearAccelWorld.z();

      // Store Previous State ---
      double xAcc_old = xAcc;
      double yAcc_old = yAcc;
      double zAcc_old = zAcc;
      double xVel_old = xVel;
      double yVel_old = yVel;
      double zVel_old = zVel;

      // Update Velocity (Trapezoidal Rule) ---
      // vel_new = vel_old + (accel_old + accel_new) / 2.0 * dt
      xVel = xVel_old + (xAcc_old + xAcc) / 2.0 * dt;
      yVel = yVel_old + (yAcc_old + yAcc) / 2.0 * dt;
      zVel = zVel_old + (zAcc_old + zAcc) / 2.0 * dt;

      // Update Position (Trapezoidal Rule) ---
      // pos_new = pos_old + (vel_old + vel_new) / 2.0 * dt
      xPos = xPos + (xVel_old + xVel) / 2.0 * dt;
      yPos = yPos + (yVel_old + yVel) / 2.0 * dt;
      zPos = zPos + (zVel_old + zVel) / 2.0 * dt;

      imu::Vector orientation = quat.toEuler();
      Serial.printf("dt:%.3f", dt);
      Serial.printf(",Yaw:%.1f", orientation.x() / DEG_2_RAD);
      Serial.printf(",Pitch:%.1f", orientation.z() / DEG_2_RAD);
      Serial.printf(",Roll:%.1f", orientation.y() / DEG_2_RAD);
      Serial.printf(",AccelX:%.1f", linearAccelBody.x());
      Serial.printf(",AccelY:%.1f", linearAccelBody.y());
      Serial.printf(",AccelZ:%.1f", linearAccelBody.z());
      Serial.printf(",AccelWorldX:%.1f", xAcc);
      Serial.printf(",AccelWorldY:%.1f", yAcc);
      Serial.printf(",AccelWorldZ:%.1f", zAcc);

      uint8_t system, gyro, accel, mag;
      system = gyro = accel = mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);

      Serial.printf(",CAL-S:%d", system);
      Serial.printf(",CAL-G:%d", gyro);
      Serial.printf(",CAL-A:%d", accel);
      Serial.printf(",CAL-M:%d", mag);

      if (accel < 1) {
        resetPosition();
      }

      Serial.printf(",VelX:%.1f", xVel);
      Serial.printf(",VelY:%.1f", yVel);
      Serial.printf(",VelZ:%.1f", zVel);
      Serial.printf(",PosX:%.1f", xPos);
      Serial.printf(",PosY:%.1f", yPos);
      Serial.printf(",PosZ:%.1f", zPos);

      Serial.println();
    }

    // Tasks should not return, but if they somehow exit the loop,
    // they should delete themselves.
    // vTaskDelete(NULL); // Not reachable in this infinite loop example
  }
}

void setup(void) {
  Serial.begin(921600);
  while (!Serial) delay(10);
  Serial.println("BNO055 3D Dead Reckoning Example");

  // --- I2C Pin Configuration (IMPORTANT: Adjust for your board!) ---
  Wire.setPins(43, 44);
  // Wire.begin(); // Use this for standard Arduinos (Uno, Nano, Mega etc.)

  if (!bno.begin()) {
    Serial.println("No BNO055 detected... Check wiring or I2C ADDR!");
    while (1)
      delay(10);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("Sensor Initialized.");

  pinMode(BOOT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BOOT_PIN), resetPosition, FALLING);

  timerBNO055Semaphore = xSemaphoreCreateBinary();
  if (timerBNO055Semaphore == NULL) {
    Serial.println("Error: Could not create BNO055 timer semaphore!");
    while (1)
      delay(10);  // Halt execution
  } else {
    Serial.println("Timer semaphore created successfully.");
    // Note: Binary semaphore starts empty. ISR must give it first.
  }

  BaseType_t taskCreated = xTaskCreatePinnedToCore(
    deferredBNO055ProcessingTask,  // Function to implement the task
    "BNO055 Data Acquisition",     // Name of the task
    4096,                          // Stack size in words (adjust as needed)
    NULL,                          // Task input parameter
    1,                             // Priority of the task (1 is reasonable for non-critical)
    &deferredBNO055TaskHandle,     // Handle to the task (optional)
    1);                            // Core ID (0 or 1) - 1 is APP_CPU, recommended for Arduino

  if (taskCreated != pdPASS) {
    Serial.println("Error: Could not create deferred task!");
    while (1)
      ;  // Halt execution
  } else {
    Serial.println("Deferred task created and pinned to Core 1.");
  }

  timerBNO055 = timerBegin(TIMER_FREQUENCY_HZ);
  if (timerBNO055 == NULL) {
    Serial.println("Error: Failed to initialize timer!");
    while (1)
      ;  // Halt
  }

  timerAttachInterrupt(timerBNO055, &onBNO055Timer);

  // CRITICAL: Enable the timer AFTER the semaphore and task are created!
  timerAlarm(timerBNO055, TIMER_ALARM_BNO055_US, true, 0);
  Serial.print("Timer configured for ");
  Serial.print(TIMER_ALARM_BNO055_US / 1000.0);
  Serial.println(" ms period.");
}

// --- Loop Function ---
void loop(void) {
  // Can be used for simple monitoring or left empty.
  // The main work is done in tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));  // Prevent loop() from hogging CPU if empty
}
