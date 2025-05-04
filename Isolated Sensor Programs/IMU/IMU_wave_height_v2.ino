#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

const uint32_t TIMER_FREQUENCY_HZ = 1000000;   // Timer frequency: 1MHz (1 tick = 1 microsecond)
const uint64_t TIMER_ALARM_BNO055_US = 10000;  // Timer alarm period: 10,000 us = 10 ms, e.g. 100Hz

volatile SemaphoreHandle_t timerBNO055Semaphore = NULL;

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

// Butterworth filter coefficients for 4th-order bandpass filter (0.125–2 Hz)
double b[] = {1.03686193e-05, 0.0, -4.14744771e-05, 0.0, 6.22117156e-05, 0.0, -4.14744771e-05, 0.0, 1.03686193e-05};
double a[] = {1.0, -7.68843421, 25.87018926, -49.75904594, 59.83785073, -46.06952679, 22.17621497, -6.10211886, 0.73487085};

// Filter state variables
double xAcc_filt[9] = {0}; // For acceleration filtering (input to the filter)
double yAcc_filt[9] = {0}; // For acceleration filtering (input to the filter)
double zAcc_filt[9] = {0}; // For acceleration filtering (input to the filter)

double highPass_b[] = {0.1, -0.1};   // High-pass filter coefficients (example)
double highPass_a[] = {1.0, -0.8};   // High-pass filter coefficients (example)
double highPass_x[2] = {0.0};  // High-pass filter state

// --- High-Pass Filter Implementation ---
double highPassFilter(double input, double *b, double *a, double *x) {
  x[0] = input - a[1] * x[1];
  double output = b[0] * x[0] + b[1] * x[1];
  x[1] = x[0];
  return output;
}

// --- Butterworth Filter Implementation ---
double butterworthFilter(double input, double *b, double *a, double *filtState) {
  // Shift states and apply filter
  for (int i = 8; i > 0; i--) {
    filtState[i] = filtState[i-1];
  }
  filtState[0] = input;

  double output = 0.0;
  for (int i = 0; i < 9; i++) {
    output += b[i] * filtState[i];
  }

  for (int i = 1; i < 9; i++) {
    output -= a[i] * filtState[i];
  }

  return output;
}

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

  Wire1.setPins(43, 44);
  Wire1.begin();

  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

  if (!bno.begin()) {
    Serial.println("No BNO055 detected... Check wiring or I2C ADDR!");
    while (1)
      delay(10);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 Sensor Initialized.");

  unsigned long lastUpdateTime = micros();
  double dt = 0;

  for (;;) {
    if (xSemaphoreTake(timerBNO055Semaphore, portMAX_DELAY) == pdTRUE) {
      imu::Quaternion quat = bno.getQuat();
      imu::Vector linearAccelBody = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

      unsigned long tStart = micros();
      dt = (tStart - lastUpdateTime) / (double)TIMER_FREQUENCY_HZ;
      lastUpdateTime = tStart;

      imu::Vector linearAccelWorld = quat.rotateVector(linearAccelBody);
      xAcc = linearAccelWorld.x();
      yAcc = linearAccelWorld.y();
      zAcc = linearAccelWorld.z();

      // Apply High-pass Filter
      xAcc = highPassFilter(xAcc, highPass_b, highPass_a, highPass_x);
      yAcc = highPassFilter(yAcc, highPass_b, highPass_a, highPass_x);
      zAcc = highPassFilter(zAcc, highPass_b, highPass_a, highPass_x);

      // Apply Butterworth Filter
      xAcc = butterworthFilter(xAcc, b, a, xAcc_filt);
      yAcc = butterworthFilter(yAcc, b, a, yAcc_filt);
      zAcc = butterworthFilter(zAcc, b, a, zAcc_filt);

      double xAcc_old = xAcc;
      double yAcc_old = yAcc;
      double zAcc_old = zAcc;
      double xVel_old = xVel;
      double yVel_old = yVel;
      double zVel_old = zVel;

      xVel = xVel_old + (xAcc_old + xAcc) / 2.0 * dt;
      yVel = yVel_old + (yAcc_old + yAcc) / 2.0 * dt;
      zVel = zVel_old + (zAcc_old + zAcc) / 2.0 * dt;

      xPos = xPos + (xVel_old + xVel) / 2.0 * dt;
      yPos = yPos + (yVel_old + yVel) / 2.0 * dt;
      zPos = zPos + (zVel_old + zVel) / 2.0 * dt;

      imu::Vector orientation = quat.toEuler();
      Serial.printf("dt:%.5f", dt);
      Serial.printf(",Yaw:%.5f", orientation.x() / DEG_2_RAD);
      Serial.printf(",Pitch:%.5f", orientation.z() / DEG_2_RAD);
      Serial.printf(",Roll:%.5f", orientation.y() / DEG_2_RAD);
      Serial.printf(",AccelX:%.5f", linearAccelBody.x());
      Serial.printf(",AccelY:%.5f", linearAccelBody.y());
      Serial.printf(",AccelZ:%.5f", linearAccelBody.z());
      Serial.printf(",AccelWorldX:%.5f", xAcc);
      Serial.printf(",AccelWorldY:%.5f", yAcc);
      Serial.printf(",AccelWorldZ:%.5f", zAcc);

      uint8_t system, gyro, accel, mag;
      bno.getCalibration(&system, &gyro, &accel, &mag);

      Serial.printf(",CAL-S:%d", system);
      Serial.printf(",CAL-G:%d", gyro);
      Serial.printf(",CAL-A:%d", accel);
      Serial.printf(",CAL-M:%d", mag);

      if (accel < 1) {
        resetPosition();
      }

      Serial.printf(",VelX:%.5f", xVel);
      Serial.printf(",VelY:%.5f", yVel);
      Serial.printf(",VelZ:%.5f", zVel);
      Serial.printf(",PosX:%.5f", xPos);
      Serial.printf(",PosY:%.5f", yPos);
      Serial.printf(",PosZ:%.5f", zPos);

      Serial.println();
    }
  }
}

void setupBNO055Task() {
  timerBNO055Semaphore = xSemaphoreCreateBinary();
  if (timerBNO055Semaphore == NULL) {
    Serial.println("Error: Could not create BNO055 timer semaphore!");
    while (1)
      delay(10);
  } else {
    Serial.println("BNO055 Timer semaphore created successfully.");
  }

  TaskHandle_t deferredBNO055TaskHandle = NULL;
  BaseType_t taskCreated = xTaskCreatePinnedToCore(
    deferredBNO055ProcessingTask,  // Function to implement the task
    "BNO055 Data Acquisition",     // Name of the task
    4096,                          // Stack size in words
    NULL,                          // Task input parameter
    1,                             // Task priority
    &deferredBNO055TaskHandle,     // Task handle
    1);                            // Core ID

  if (taskCreated != pdPASS) {
    Serial.println("Error: Could not create deferred BNO055 task!");
    while (1)
      ;  // Halt execution
  }

  hw_timer_t* timerBNO055 = NULL;
  timerBNO055 = timerBegin(TIMER_FREQUENCY_HZ);
  if (timerBNO055 == NULL) {
    Serial.println("Error: Failed to initialize BNO055 timer!");
    while (1)
      ;  // Halt
  }

  timerAttachInterrupt(timerBNO055, &onBNO055Timer);
  timerAlarm(timerBNO055, TIMER_ALARM_BNO055_US, true, 0);

  Serial.printf("BNO055 timer configured for %.1fms period.\n", TIMER_ALARM_BNO055_US / 1000.0);
}

void setup(void) {
  Serial.begin(921600);
  while (!Serial) delay(10);
  Serial.println("BNO055 3D Dead Reckoning Example");

  pinMode(BOOT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BOOT_PIN), resetPosition, FALLING);

  setupBNO055Task();
}

void loop(void) {
  vTaskDelay(pdMS_TO_TICKS(1000));  // Prevent loop() from hogging CPU
}
