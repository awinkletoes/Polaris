#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h> // Include for sin() and cos()

// --- World frame state variables ---
volatile double xPos = 0.0; // Position in world X frame (e.g., meters)
volatile double yPos = 0.0; // Position in world Y frame (e.g., meters)
volatile double zPos = 0.0; // Position in world Z frame (e.g., meters)
volatile double xVel = 0.0; // Velocity in world X frame (e.g., m/s)
volatile double yVel = 0.0; // Velocity in world Y frame (e.g., m/s)
volatile double zVel = 0.0; // Velocity in world Z frame (e.g., m/s)

// --- Timing and Constants ---
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; // How often to read data (aiming for 100Hz)
uint16_t PRINT_RATE_MS = 100; // How often to print data (e.g., every 100ms)
uint16_t printCounter = 0; // Counter for printing

// Time delta in seconds, derived from sample rate
double dt = (double)BNO055_SAMPLERATE_DELAY_MS / 1000.0;

// Conversion factor
double DEG_2_RAD = 0.01745329251; // Degrees to Radians

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void IRAM_ATTR resetPosition() {
xVel = yVel = zVel = xPos = yPos = zPos = 0;
}

// --- Helper Function (Optional but Recommended) ---
// Function to display sensor calibration status
void displayCalStatus(void) {
uint8_t system, gyro, accel, mag;
system = gyro = accel = mag = 0;
bno.getCalibration(&system, &gyro, &accel, &mag);
Serial.print("CALIBRATION: Sys=");
Serial.print(system, DEC);
Serial.print(" Gyro=");
Serial.print(gyro, DEC);
Serial.print(" Accel=");
Serial.print(accel, DEC);
Serial.print(" Mag=");
Serial.println(mag, DEC);

// Reset data when the system is not fully calibrated
if (system < 3) {
resetPosition();
}
}

// --- Setup Function ---
void setup(void) {
Serial.begin(921600);
while (!Serial) delay(10); // wait for serial port to open!
Serial.println("BNO055 3D Dead Reckoning Example");
Serial.println("Current time: Friday, March 28, 2025 at 12:46:15 PM (Milwaukee, WI)");
Serial.println("");

// --- I2C Pin Configuration (IMPORTANT: Adjust for your board!) ---
// Standard pins for most Arduinos are Wire.begin() without arguments.
// Use setPins only if needed (e.g., ESP32) and use the correct GPIOs.
Wire.setPins(43, 44); // Example for specific boards like ESP32
// Wire.begin(); // Use this for standard Arduinos (Uno, Nano, Mega etc.)

// Initialize BNO055 sensor
if (!bno.begin()) {
Serial.print("No BNO055 detected... Check wiring or I2C ADDR!");
while (1)
; // Halt execution
}

delay(1000); // Allow sensor to stabilize

// Set sensor to NDOF mode (uses fusion) for best orientation data
bno.setExtCrystalUse(true); // Use external crystal for better accuracy

Serial.println("Sensor Initialized.");
displayCalStatus(); // Display initial calibration status
Serial.println("--------------------------------------------------");

pinMode(BOOT_PIN, INPUT);
attachInterrupt(digitalPinToInterrupt(BOOT_PIN), resetPosition, FALLING);
}

// --- Loop Function ---
void loop(void) {
unsigned long tStart = micros(); // Record start time of loop iteration

// --- 1. Read Sensor Data ---
sensors_event_t orientationData, linearAccelData;
// Get Euler angles (degrees) and linear acceleration (m/s^2)
bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

// --- 2. Extract Data and Convert Angles to Radians ---
// Euler angles from sensor (check library documentation for specific order if unsure)
// Assuming: x=Yaw (psi), y=Roll (phi), z=Pitch (theta)
double psi_rad = orientationData.orientation.x * DEG_2_RAD; // Yaw
double phi_rad = orientationData.orientation.y * DEG_2_RAD; // Roll
double theta_rad = orientationData.orientation.z * DEG_2_RAD; // Pitch

// Linear acceleration in sensor's body frame
double sensorAx = linearAccelData.acceleration.x;
double sensorAy = linearAccelData.acceleration.y;
double sensorAz = linearAccelData.acceleration.z;

// --- 3. Calculate Sines and Cosines for Rotation ---
double c_psi = cos(psi_rad);
double s_psi = sin(psi_rad);
double c_phi = cos(phi_rad);
double s_phi = sin(phi_rad);
double c_theta = cos(theta_rad);
double s_theta = sin(theta_rad);

// --- 4. Rotate Acceleration Vector from Body Frame to World Frame ---
// Using ZYX rotation sequence (Yaw-Pitch-Roll)
// R = Rz(psi) * Ry(theta) * Rx(phi)
// world_accel = R * body_accel

double accelWorldX = sensorAx * (c_psi * c_theta) + sensorAy * (c_psi * s_theta * s_phi - s_psi * c_phi) + sensorAz * (c_psi * s_theta * c_phi + s_psi * s_phi);

double accelWorldY = sensorAx * (s_psi * c_theta) + sensorAy * (s_psi * s_theta * s_phi + c_psi * c_phi) + sensorAz * (s_psi * s_theta * c_phi - c_psi * s_phi);

double accelWorldZ = sensorAx * (-s_theta) + sensorAy * (c_theta * s_phi) + sensorAz * (c_theta * c_phi);

// --- 5. Store Previous State ---
double xVel_old = xVel;
double yVel_old = yVel;
double zVel_old = zVel;

// --- 6. Update Velocity (Explicit Euler) ---
// vel_new = vel_old + accel_world * dt
xVel = xVel_old + accelWorldX * dt;
yVel = yVel_old + accelWorldY * dt;
zVel = zVel_old + accelWorldZ * dt;

// --- 7. Update Position (Trapezoidal Rule) ---
// pos_new = pos_old + (vel_old + vel_new) / 2.0 * dt
xPos = xPos + (xVel_old + xVel) / 2.0 * dt;
yPos = yPos + (yVel_old + yVel) / 2.0 * dt;
zPos = zPos + (zVel_old + zVel) / 2.0 * dt;


// --- 8. Print Data Periodically ---
if (printCounter * BNO055_SAMPLERATE_DELAY_MS >= PRINT_RATE_MS) {
Serial.print("Orient(YPR): ");
Serial.print(orientationData.orientation.x, 1);
Serial.print(", "); // Yaw
Serial.print(orientationData.orientation.z, 1);
Serial.print(", "); // Pitch
Serial.print(orientationData.orientation.y, 1); // Roll
Serial.print(" | Accel(World): ");
Serial.print(accelWorldX, 2);
Serial.print(", ");
Serial.print(accelWorldY, 2);
Serial.print(", ");
Serial.print(accelWorldZ, 2);
Serial.print(" | Vel(World): ");
Serial.print(xVel, 2);
Serial.print(", ");
Serial.print(yVel, 2);
Serial.print(", ");
Serial.print(zVel, 2);
Serial.print(" | Pos(World): ");
Serial.print(xPos, 2);
Serial.print(", ");
Serial.print(yPos, 2);
Serial.print(", ");
Serial.print(zPos, 2);
Serial.println();

// Note that a side effect of calling this is currently that data is reset when the system is not fully calibrated
displayCalStatus();
// Serial.println("-------");

printCounter = 0; // Reset print counter
} else {
printCounter++;
}

// --- 9. Enforce Sample Rate ---
// Busy-wait until the desired sample time has elapsed
while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
// Do nothing, just wait
}
}