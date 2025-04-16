#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Constants
const float dt = 0.01; // 100 Hz sample rate
const int sampleCount = 600; // 6 seconds * 100 Hz

// Butterworth filter coefficients (bandpass: 0.125–2 Hz)
const float b[] = {
  1.03686193e-05, 0.0, -4.14744771e-05, 0.0,
  6.22117156e-05, 0.0, -4.14744771e-05, 0.0,
  1.03686193e-05
};
const float a[] = {
  1.0, -7.68843421, 25.87018926, -49.75904594,
  59.83785073, -46.06952679, 22.17621497, -6.10211886,
  0.73487085
};

// Filter state buffers for bandpass filter
float x_history[9] = {0};  // Input history
float y_history[9] = {0};  // Output history

// Raw and processed buffers
float accelZ_raw[sampleCount];
float velZ[sampleCount];
float posZ[sampleCount];

// Velocity & position state
float velocity = 0;
float position = 0;

// Min/max trackers
float maxHeight = -1000;
float minHeight = 1000;

// Velocity low-pass & high-pass filter state
float velLPF = 0;
float velHPF = 0;
float prevVel = 0;
float prevHPF = 0;

// Filter parameters
const float velLowCut = 0.125;  // LPF cutoff (Hz)
const float velHighCut = 2.0;   // HPF cutoff (Hz)

float calculateAlpha(float cutoffFreq) {
  float tau = 1.0 / (2 * 3.14159 * cutoffFreq);
  return dt / (tau + dt);
}

// Apply custom 8th-order IIR bandpass filter
float applyBandpassFilter(float input) {
  // Clamp extreme input values to avoid blowing up the filter
  if (isnan(input) || isinf(input)) input = 0;
  if (input > 100.0) input = 100.0;
  if (input < -100.0) input = -100.0;

  // Shift histories
  for (int i = 8; i > 0; i--) {
    x_history[i] = x_history[i - 1];
    y_history[i] = y_history[i - 1];
  }
  x_history[0] = input;

  // Compute output using Direct Form II Transposed
  float result = b[0] * x_history[0];
  for (int i = 1; i < 9; i++) {
    result += b[i] * x_history[i];
    result -= a[i] * y_history[i];
  }

  // Check result sanity
  if (isnan(result) || isinf(result) || fabs(result) > 100.0) {
    result = 0;
  }

  y_history[0] = result;
  return result;
}


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.setPins(43, 44);  // Optional for custom I2C pins
  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }

  bno.setExtCrystalUse(true);
}

void loop() {
  Serial.println("=== Collecting 6s of Data ===");

memset(x_history, 0, sizeof(x_history));
memset(y_history, 0, sizeof(y_history));


  // Reset values
  velocity = 0;
  position = 0;
  maxHeight = -1000;
  minHeight = 1000;
  prevVel = 0;
  prevHPF = 0;

  float alphaLPF = calculateAlpha(velLowCut);
  float alphaHPF = calculateAlpha(velHighCut);

  for (int i = 0; i < sampleCount; i++) {
    sensors_event_t event;
    bno.getEvent(&event);

    float rawZ = event.acceleration.z;
    accelZ_raw[i] = rawZ;

    // Apply bandpass filter to Z-acceleration
    float filteredAccel = applyBandpassFilter(rawZ);

  if (isnan(filteredAccel) || isinf(filteredAccel) || fabs(filteredAccel) > 100.0) {
  Serial.println("Filtered Acceleration overflow detected! Resetting.");
  filteredAccel = 0;
}


    // Integrate acceleration -> velocity (Euler)
    velocity += filteredAccel * dt;

    // Low-pass filter velocity
    velLPF = alphaLPF * velocity + (1.0 - alphaLPF) * velLPF;

    // High-pass filter velocity
    velHPF = alphaHPF * (velLPF - prevVel) + (1.0 - alphaHPF) * prevHPF;

    prevVel = velLPF;
    prevHPF = velHPF;

    velZ[i] = velHPF;

    // Integrate velocity -> position (trapezoidal)
    position += velHPF * dt;
    posZ[i] = position;

    // Track wave height
    if (position > maxHeight) maxHeight = position;
    if (position < minHeight) minHeight = position;

    delay(10);  // 100 Hz
  }

  // Report results
  Serial.println("\n=== Wave Height Summary ===");
  Serial.print("Max height: "); Serial.println(maxHeight, 4);
  Serial.print("Min height: "); Serial.println(minHeight, 4);
  Serial.print("Wave height (peak-to-peak): "); Serial.println(maxHeight - minHeight, 4);
  Serial.println("============================\n");

  delay(1000); // Delay before repeating
}
