// IMU_wave_height_v3.ino - Updated with signal detrending and quaternion rotation fix
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int MAX_SAMPLES = 600;
float accZ_buffer[MAX_SAMPLES];
float z_filtered[MAX_SAMPLES];
float velocity[MAX_SAMPLES];
float position[MAX_SAMPLES];

unsigned long startTime;
int sampleIndex = 0;
bool dataReady = false;

// Quaternion-based frame rotation
imu::Vector<3> rotateVectorByQuaternion(const imu::Vector<3>& v, const imu::Quaternion& q) {
  float w = q.w();
  float x = q.x();
  float y = q.y();
  float z = q.z();

  float vx = v.x();
  float vy = v.y();
  float vz = v.z();

  float qw2 = w*w, qx2 = x*x, qy2 = y*y, qz2 = z*z;

  float r00 = qw2 + qx2 - qy2 - qz2;
  float r01 = 2 * (x*y - w*z);
  float r02 = 2 * (x*z + w*y);
  float r10 = 2 * (x*y + w*z);
  float r11 = qw2 - qx2 + qy2 - qz2;
  float r12 = 2 * (y*z - w*x);
  float r20 = 2 * (x*z - w*y);
  float r21 = 2 * (y*z + w*x);
  float r22 = qw2 - qx2 - qy2 + qz2;

  float rx = r00 * vx + r01 * vy + r02 * vz;
  float ry = r10 * vx + r11 * vy + r12 * vz;
  float rz = r20 * vx + r21 * vy + r22 * vz;

  return imu::Vector<3>(rx, ry, rz);
}

void setup() {
  Serial.begin(115200);
  Wire.setPins(43,44);
  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);
  Serial.println("Starting IMU Wave Height Measurement...");
}

void loop() {
  if (sampleIndex == 0) {
    Serial.println("\nCollecting 6 seconds of data...");
    startTime = millis();
  }

  sensors_event_t event;
  bno.getEvent(&event);

  imu::Quaternion q = bno.getQuat();
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> acc_earth = rotateVectorByQuaternion(acc, q);

  accZ_buffer[sampleIndex] = acc_earth.z();
  sampleIndex++;

  delay(10); // 100 Hz

  if (sampleIndex >= MAX_SAMPLES) {
    dataReady = true;
    sampleIndex = 0;
  }

  if (dataReady) {
    processWaveHeight();
    dataReady = false;
  }
}

void processWaveHeight() {
  Serial.println("\nProcessing wave height...");

  // Simple smoothing filter
  for (int i = 1; i < MAX_SAMPLES - 1; i++) {
    z_filtered[i] = 0.25 * accZ_buffer[i - 1] + 0.5 * accZ_buffer[i] + 0.25 * accZ_buffer[i + 1];
  }
  z_filtered[0] = accZ_buffer[0];
  z_filtered[MAX_SAMPLES - 1] = accZ_buffer[MAX_SAMPLES - 1];

  // --- Detrend (remove mean) ---
  float meanZ = 0.0;
  for (int i = 0; i < MAX_SAMPLES; i++) {
    meanZ += z_filtered[i];
  }
  meanZ /= MAX_SAMPLES;
  for (int i = 0; i < MAX_SAMPLES; i++) {
    z_filtered[i] -= meanZ;
  }

  // Integrate to velocity
  float dt = 0.01; // 100 Hz
  velocity[0] = 0.0;
  for (int i = 1; i < MAX_SAMPLES; i++) {
    velocity[i] = velocity[i - 1] + 0.5 * (z_filtered[i] + z_filtered[i - 1]) * dt;
  }

  // Integrate velocity to position
  position[0] = 0.0;
  for (int i = 1; i < MAX_SAMPLES; i++) {
    position[i] = position[i - 1] + 0.5 * (velocity[i] + velocity[i - 1]) * dt;
  }

  Serial.println("Index\tAccZ\t	Velocity\tPosition");
  for (int i = 0; i < MAX_SAMPLES; i++) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(z_filtered[i], 2);
    Serial.print("\t");
    Serial.print(velocity[i], 2);
    Serial.print("\t");
    Serial.println(position[i], 2);
  }

  // Estimate wave height
  float minPos = position[0], maxPos = position[0];
  for (int i = 1; i < MAX_SAMPLES; i++) {
    if (position[i] < minPos) minPos = position[i];
    if (position[i] > maxPos) maxPos = position[i];
  }
  float waveHeight = maxPos - minPos;

  if (isnan(waveHeight) || waveHeight < 0.01 || waveHeight > 10.0) {
    Serial.println("Wave height too small or invalid.");
  } else {
    Serial.print("Estimated Wave Height: ");
    Serial.print(waveHeight, 2);
    Serial.println(" meters");
  }
}
