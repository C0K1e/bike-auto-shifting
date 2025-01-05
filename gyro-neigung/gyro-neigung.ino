#include <Wire.h>

// MPU-6050 I2C address
const int MPU_ADDR = 0x68;

// Accelerometer and Gyroscope raw values
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

// Scaling factors
const float accelScale = 2.0 / 32768.0;  // For ±2g range
const float maxTiltDegrees = 45.0;      // Maximum tilt degrees for 100% tilt

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power Management 1 register
  Wire.write(0);    // Wake up MPU-6050
  Wire.endTransmission(true);

  Serial.println("MPU-6050 Initialized.");
}

void loop() {
  float tiltDegrees = getTiltDegrees();
  float tiltPercentage = (tiltDegrees / maxTiltDegrees) * 100;

  Serial.print("Tilt in Degrees: ");
  Serial.print(tiltDegrees);
  Serial.print("°\t");

  Serial.print("Tilt in Percentage: ");
  Serial.print(tiltPercentage);
  Serial.println("%");

  delay(1000); // Update every second
}

// Function to calculate tilt angle in degrees
float getTiltDegrees() {
  // Request accelerometer and gyroscope data from MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  // Read accelerometer values
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read(); // Reading Gyro, even if not used here

  // Convert raw accelerometer data to g
  float ax = accelX * accelScale;
  float ay = accelY * accelScale;
  float az = accelZ * accelScale;

  // Calculate tilt angle in degrees
  float accelAngle = atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;

  return accelAngle;
}
