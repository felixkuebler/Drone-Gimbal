/**
* Mini Drone Gimbal
*/

#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>

// I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
constexpr int MPU_ADDR = 0x68; 

float gyroAngleX;
float gyroAngleY;

float elapsedTime;
float currentTime;
float previousTime;

// Servo signal GPIOs
constexpr uint8_t servoRollPin = 10;
constexpr uint8_t servoPitchPin = 9;

Servo servoRollAxis;
Servo servoPitchAxis;

constexpr int16_t servoOffsetRollAngle = 17;
constexpr int16_t servoOffsetPitchAngle = 17;

constexpr float filterWeightRoll = 0.5;
float filterRollAngle = 0;
uint8_t rollAngle = 0;

constexpr float filterWeightPitch = 0.5;
float filterPitchAngle = 0;
uint8_t pitchAngle = 0;

void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  servoRollAxis.attach(servoRollPin);
  servoPitchAxis.attach(servoPitchPin);
}

void loop() {

  // Input
  // Read accelerometer data  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  float accX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  float accY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  float accZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  float accAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI) + 0.43; // AccErrorX ~(-0.43) See the calculate_IMU_error()custom function for more details
  float accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI) + 0.69; // AccErrorY ~(-0.69)
  
  // Read gyroscope data
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  // Read gyroscope data
  // Gyro data first register address 0x43
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  float gyroX = (Wire.read() << 8 | Wire.read()) / 131.0; 
  float gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  gyroX = gyroX + 1.48; // GyroErrorX ~(-1.48)
  gyroY = gyroY - 0.85; // GyroErrorY ~(0.85)
  // Currently the raw values are in degrees per seconds, 
  //deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + gyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + gyroY * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  float gyroRollAngle = 0.96 * gyroAngleX + 0.04 * accAngleX;
  float gyroPitchAngle = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // Print the values on the serial monitor
  /*
  Serial.print(gyroRollAngle);
  Serial.print("/");
  Serial.println(gyroPitchAngle);
  */
  // Control Loop

  const int16_t desiredRollAngle = 90;
  int16_t errorRollAngle = desiredRollAngle - gyroRollAngle;
  errorRollAngle = errorRollAngle > 135 ? 135 : errorRollAngle;
  errorRollAngle = errorRollAngle < 45 ? 45 : errorRollAngle;

  const int16_t desiredPitchAngle = 90;
  int16_t errorPitchAngle = desiredPitchAngle - gyroPitchAngle;
  errorPitchAngle = errorPitchAngle > 135 ? 135 : errorPitchAngle;
  errorPitchAngle = errorPitchAngle < 45 ? 45 : errorPitchAngle;

  // Filter

  filterRollAngle = filterWeightRoll*(float)errorRollAngle + (1-filterWeightRoll)*filterRollAngle;
  filterPitchAngle = filterWeightPitch*(float)errorPitchAngle + (1-filterWeightPitch)*filterPitchAngle;

  // Output

  if (filterRollAngle != rollAngle) {
    rollAngle = filterRollAngle;
    servoRollAxis.write((uint8_t)(filterRollAngle + servoOffsetRollAngle));
  }

  if (filterPitchAngle != pitchAngle) {
    pitchAngle = filterPitchAngle;
    servoPitchAxis.write((uint8_t)(filterPitchAngle + servoOffsetPitchAngle));
  }

  delay(5);
}
